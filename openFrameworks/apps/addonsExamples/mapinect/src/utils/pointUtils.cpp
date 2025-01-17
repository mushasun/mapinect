#include "pointUtils.h"

#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/range_image/range_image.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/transforms.h>

#include "ofxKinect.h"
#include "ofxOpenCv.h"

#include "Constants.h"
#include "Feature.h"
#include "Globals.h"
#include "Line2D.h"
#include "Model.h"
#include <algorithm>
#include "Table.h"
#include "utils.h"
#include "Feature.h"
#include "Plane3D.h"
#include "transformationUtils.h"

// -------------------------------------------------------------------------------------
// pcl <--> ofVec3f and other conversion utils
// -------------------------------------------------------------------------------------

void setPointXYZ(PCXYZ& p, float x, float y, float z)
{
	p.x = x;
	p.y = y;
	p.z = z;
}

vector<ofVec3f> pointCloudToOfVecVector(const PCPtr& cloud)
{
	vector<ofVec3f> result;
	for (int k = 0; k < cloud->size(); k++)
	{
		result.push_back(PCXYZ_OFVEC3F(cloud->at(k)));
	}
	return result;
}

PCPtr ofVecVectorToPointCloud(const vector<ofVec3f>& v)
{
	PCPtr result(new PC());
	for (int k = 0; k < v.size(); k++)
	{
		result->push_back(OFVEC3F_PCXYZ(v.at(k)));
	}
	return result;
}

PCPtr getCloudFromIndices(const PCPtr& cloud, const pcl::PointIndices& pi)
{
	PCPtr newCloud(new PC());
	
	for (vector<int>::const_iterator pit = pi.indices.begin(); pit != pi.indices.end(); pit++)
		newCloud->points.push_back(cloud->points[*pit]);
	
	return newCloud;
}


// -------------------------------------------------------------------------------------
// i/o utils
// -------------------------------------------------------------------------------------

bool saveCloud(const string& filename, const PC& cloud)
{
	if (!mapinect::IsFeatureSaveCloudActive())
		return false;
	if (cloud.empty())
		return false;
	
	if(cloud.width * cloud.height != cloud.points.size())
	{
		PC printeableCloud (cloud);
		printeableCloud.height = 1;
		printeableCloud.width = cloud.points.size();
		pcl::io::savePCDFileASCII (filename, printeableCloud);
	}
	else
	{
		pcl::io::savePCDFileASCII (filename, cloud);
	}
	return true;
}

void saveCloud(const string& filename, const ofVec3f& p)
{
	if (mapinect::IsFeatureSaveCloudActive())
	{
		PCPtr cloud (new PC());
		cloud->push_back(OFVEC3F_PCXYZ(p));

		saveCloud(filename, *cloud);
	}
}

void saveCloud(const string& filename, const vector<ofVec3f>& v)
{
	if (mapinect::IsFeatureSaveCloudActive())
	{
		PCPtr cloud(ofVecVectorToPointCloud(v));
		saveCloud(filename, *cloud);
	}
}

PCPtr loadCloud(const string& filename)
{
	PCPtr cloud(new PC());
	pcl::io::loadPCDFile<PCXYZ>(filename, *cloud);
	return cloud;
}

PCPtr getCloud(const ofVec3f& min, const ofVec3f& max, int stride)
{
	gTransformation->cloudMutex.lock();
	PCPtr t = getCloudWithoutMutex(min,max,stride);
	gTransformation->cloudMutex.unlock();
	return t;
}

PCPtr getCloudWithoutMutex(const ofVec3f& min, const ofVec3f& max, int stride)
{
	if (min.x < 0 || min.y < 0
		|| max.x > KINECT_DEFAULT_WIDTH || max.y > KINECT_DEFAULT_HEIGHT
		|| min.x > max.x || min.y > max.y)
	{
		PCL_ERROR ("Wrong arguments to obtain the cloud\n");
		return PCPtr(new PC());
	}

	float voxelSize = mapinect::Constants::CLOUD_VOXEL_SIZE;
	if(mapinect::IsFeatureUniformDensityActive())
	{
		voxelSize = mapinect::Constants::CLOUD_VOXEL_SIZE_FOR_STRIDE(stride);
		stride = 1;
	}
	const int minStride = ::min(mapinect::Constants::CLOUD_STRIDE_H, mapinect::Constants::CLOUD_STRIDE_V);
	const int hStride = stride + mapinect::Constants::CLOUD_STRIDE_H - 1;
	const int vStride = stride + mapinect::Constants::CLOUD_STRIDE_V - 1;

	// Allocate space for max points available
	PCPtr partialCloud(new PC());
	partialCloud->width    = ceil((max.x - min.x) / hStride);
	partialCloud->height   = ceil((max.y - min.y) / vStride);
	partialCloud->is_dense = false;
	partialCloud->points.resize(partialCloud->width * partialCloud->height);
	register float* depthMap = gKinect->getDistancePixels();
	// Iterate over the image's rectangle with step = stride
	register int depthIndex = 0;
	int cloudIndex = 0;
	for(int v = min.y; v < max.y; v += vStride) {
		for(register int u = min.x; u < max.x; u += hStride) {
			depthIndex = v * KINECT_DEFAULT_WIDTH + u;

			PCXYZ& pt = partialCloud->points[cloudIndex];
			cloudIndex++;

			// Check for invalid measures
			if(depthMap[depthIndex] == 0)
			{
				pt.x = pt.y = pt.z = 0;
			}
			else
			{
				ofVec3f pto = gKinect->getWorldCoordinateFor(u,v);

				if(pto.z > mapinect::Constants::CLOUD_Z_MAX)
				{
					pt.x = pt.y = pt.z = 0;
				}
				else
				{
					pt.x = pto.x;
					pt.y = pto.y;
					pt.z = pto.z;
				}
			}
		}
	}

	if(mapinect::IsFeatureUniformDensityActive())
	{
		PCPtr preFilter(new PC(*partialCloud));
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud (preFilter);
		sor.setLeafSize (voxelSize, voxelSize, voxelSize);
		sor.filter(*partialCloud);
	}

	return transformCloud(partialCloud, gTransformation->getWorldTransformation());
}

PCPtr getCloudWithoutMutex(int stride)
{
	return getCloudWithoutMutex(
		ofPoint(0, 0),
		ofPoint(KINECT_DEFAULT_WIDTH, KINECT_DEFAULT_HEIGHT),
		stride);
}

PCPtr getCloud(int stride)
{
	gTransformation->cloudMutex.lock();
	PCPtr t = getCloudWithoutMutex(stride);
	gTransformation->cloudMutex.unlock();
	return t;
}


PCPtr getCloudWithoutMutex()
{
	return getCloudWithoutMutex(mapinect::Constants::CLOUD_STRIDE());
}

PCPtr getCloud()
{
	gTransformation->cloudMutex.lock();
	PCPtr t = getCloudWithoutMutex(mapinect::Constants::CLOUD_STRIDE());
	gTransformation->cloudMutex.unlock();
	return t;
}

// -------------------------------------------------------------------------------------
// plane utils
// -------------------------------------------------------------------------------------

ofVec3f getNormal(const pcl::ModelCoefficients& coefficients)
{
	return ofVec3f(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
}


PCPtr findPlaneGivenNormal(const PCPtr& cloud, pcl::ModelCoefficients& coefficients, 
							ofVec3f normal, float angleThreshold, float distanceThreshold, int maxIterations ) {
	// http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
	// http://www.pcl-users.org/Ransac-Planes-td2085912.html#a2086759
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::SACSegmentation<pcl::PointXYZ> seg;  
	seg.setOptimizeCoefficients (false);  
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);  
	seg.setMethodType (pcl::SAC_RANSAC);  
	seg.setMaxIterations (maxIterations);  
	seg.setDistanceThreshold (distanceThreshold);  //  How close a point must be to the model in order to be considered an inlier
	cout << "Buscando planos con normal = " << normal << endl;
	seg.setAxis(Eigen::Vector3f(normal.x,normal.y,normal.z));  
	float angleThresholdRad = ofDegToRad(angleThreshold);
	seg.setEpsAngle(angleThresholdRad);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, coefficients);
	return getCloudFromIndices(cloud, *inliers);
}


PCPtr extractBiggestPlane(const PCPtr& cloud, pcl::ModelCoefficients& coefficients, PCPtr& remainingCloud,
							float distanceThreshold, int maxIterations)
{
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::SACSegmentation<PCXYZ> seg;
	pcl::ExtractIndices<PCXYZ> extract;

	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxIterations);
	seg.setDistanceThreshold(distanceThreshold);

	// Create the filtering object
	int i = 0;
	int nrPoints = cloud->points.size ();
	seg.setInputCloud(cloud);
	seg.segment(*inliers, coefficients);
	if (inliers->indices.size () == 0)
	{
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		PCPtr empty(new PC());
		return empty;
	}

	PCPtr result(new PC());

	if (inliers->indices.size() != cloud->size())
	{
		// Fragment the cloud into plane cloud and the remaining cloud
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*result);

		// Remove plane's points
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*remainingCloud);
	}
	else
	{
		result = cloud;
		remainingCloud = PCPtr(new PC());
	}

	return result;
}

pcl::PointIndices::Ptr adjustPlane(const pcl::ModelCoefficients& coefficients, const PCPtr& cloudToAdjust)
{
	float PLANE_THRESHOLD = 0.009;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	for (int k = 0; k < cloudToAdjust->size(); k++) {
		PCXYZ pto = cloudToAdjust->at(k);
		float val = coefficients.values[0] * pto.x + 
					coefficients.values[1] * pto.y + 
					coefficients.values[2] * pto.z + 
					coefficients.values[3]; 

		if(abs(val) < PLANE_THRESHOLD)
			inliers->indices.push_back(k);
	}
	return inliers;
}

float evaluatePoint(const pcl::ModelCoefficients& coefficients, const PCXYZ& p)
{
	return coefficients.values.at(0) * p.x +
		   coefficients.values.at(1) * p.y +
		   coefficients.values.at(2) * p.z +
		   coefficients.values.at(3);
}

float evaluatePoint(const pcl::ModelCoefficients& coefficients, const ofVec3f& p)
{
	return coefficients.values.at(0) * p.x +
		   coefficients.values.at(1) * p.y +
		   coefficients.values.at(2) * p.z +
		   coefficients.values.at(3);
}

PCPtr projectPointsInPlane(const PCPtr& points, const pcl::ModelCoefficients& plane)
{
	mapinect::Plane3D plane3d(plane);
	vector<ofVec3f>	ptos = pointCloudToOfVecVector(points);
	PCPtr result (new PC());
	for(int i = 0; i < ptos.size(); i ++)
	{
		result->push_back(OFVEC3F_PCXYZ(plane3d.project(ptos.at(i))));
	}
	return result;
}

vector<ofVec3f> projectPointsInPlane(const vector<Eigen::Vector3f>& points, const pcl::ModelCoefficients& plane)
{
	vector<ofVec3f> result;
	vector<ofVec3f> ptos = eigenVectorToOfVecVector(points);

	mapinect::Plane3D plane3d(plane);
	for(int i = 0; i < ptos.size(); i ++)
	{
		result.push_back(plane3d.project(ptos.at(i)));
	}
	return result;
}

// -------------------------------------------------------------------------------------
// objects recognition utils
// -------------------------------------------------------------------------------------

ObjectType getObjectType(const PCPtr& cloud)
{
	float boxProb = boxProbability(cloud);

	ObjectType ret;
	
	if(boxProb > .50)
		return ret = BOX;
	else
		return ret = UNRECOGNIZED;
}

float boxProbability(const PCPtr& cloud)
{
	gModel->tableMutex.lock();
	mapinect::TablePtr table = gModel->getTable();
	bool isCloudOnTable = false;
	ofVec3f tableNormal;
	if (table != NULL)
	{
		isCloudOnTable = table->isOnTable(cloud);
		tableNormal = table->getNormal();
	}
	gModel->tableMutex.unlock();
	if(isCloudOnTable)
	{
		//pcl::io::savePCDFile("isBox.pcd",*cloud);
		vector<ofVec3f> normals;
		float totalPoints = cloud->size();
		float pointsInPlanes = 0;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
		pcl::SACSegmentation<PCXYZ> seg;
		pcl::ExtractIndices<PCXYZ> extract;
		PCPtr cloudP (new PC());
		PCPtr cloudTemp (new PC(*cloud));
	
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (50);
		seg.setDistanceThreshold (0.009); //original: 0.01

		float DOT_EPSILON = 0.15;

		// Create the filtering object
		int nrPoints = cloud->points.size ();

		int numFaces = 0;
		while (cloudTemp->points.size () > 0.07 * nrPoints && numFaces < 8)
		{
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud (cloudTemp);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0) {
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}

			pointsInPlanes += inliers->indices.size();

			//FIX
			PCPtr cloudFilteredTempInliers (new PC());
			PCPtr cloudFilteredTempOutliers (new PC());
			if(inliers->indices.size() != cloudTemp->size())
			{
				// Extract the inliers
				extract.setInputCloud (cloudTemp);
				extract.setIndices (inliers);
				extract.setNegative (false);
				extract.filter (*cloudFilteredTempInliers);
				cloudP = cloudFilteredTempInliers;
			}
			else
				cloudP = cloudTemp;
		
			ofVec3f norm (coefficients->values[0],coefficients->values[1],coefficients->values[2]);
			norm.normalize();

			//Chequeo que las normales sean perpendiculares entre si y paraleas o perpendiculares a la mesa.
			if(table != NULL)
			{
				float dot = abs(tableNormal.dot(norm));
				if( dot > DOT_EPSILON && dot < (1.0 - DOT_EPSILON))
				{
					//cout << "plane is neither orthogonal nor parallel, dot: " << dot << " normal: " << norm << endl;
					return 0;
				}
				//si es paralela a la mesa, chequeo que est� sobre la mesa
				if(dot > (1.0 - DOT_EPSILON))
				{
					if(!table->isOverTable(cloudP))
					{
						//cout << "plane not over table" << endl;
						return 0;
					}
				}
			}
			for(int i = 0; i < normals.size(); i++)
			{
				float dot = abs(normals[i].dot(norm));
				if( dot > DOT_EPSILON)
				{
					//cout << "plane normals aren't orthogonal, dot: " << dot << " normal A: " << norm << " normal B: " << normals[i] << endl;
					return 0;
				}
			}

			normals.push_back(norm);
			// Create the filtering object
			extract.setInputCloud (cloudTemp);
			extract.setIndices (inliers);
			extract.setNegative (true);

		
			if(cloudP->size() != cloudTemp->size())
				extract.filter (*cloudFilteredTempOutliers);

			cloudTemp = cloudFilteredTempOutliers;

			numFaces++;
		}

		float pointsPercent = pointsInPlanes/totalPoints;
		if (inRange(numFaces, 2, 3))
			return pointsPercent;
		else if (numFaces == 1 && abs(tableNormal.dot(normals[0])) < DOT_EPSILON)
			return pointsPercent;
		else
			return 0;
	}
	else
		return 0;
}

vector<ofVec3f> findRectangle(const PCPtr& cloud, const pcl::ModelCoefficients& coefficients) 
{ 
	vector<Eigen::Vector3f> corners; 
	//saveCloudAsFile("toproject.pcd",*cloud);
	pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients(coefficients));
	// Project points onto the table plane 

	PC projectedCloud = *cloud; 

	//saveCloudAsFile("projected.pcd",projectedCloud);

	PCXYZ pto = cloud->at(1);
	float val = coefficients.values[0] * pto.x + 
				coefficients.values[1] * pto.y + 
				coefficients.values[2] * pto.z + 
				coefficients.values[3]; 

	if(abs(val) < 0.009)
		;;

	// store the table top plane parameters 
	Eigen::Vector3f planeNormal; 
	planeNormal.x() = coeff->values[0]; 
	planeNormal.y() = coeff->values[1]; 
	planeNormal.z() = coeff->values[2]; 
	
	// compute an orthogonal normal to the plane normal 
	Eigen::Vector3f v = planeNormal.unitOrthogonal(); 
	
	// take the cross product of the two normals to get 
	// a thirds normal, on the plane 
	Eigen::Vector3f u = planeNormal.cross(v); 

	// project the 3D point onto a 2D plane 
	std::vector<cv::Point2f> points; 
	
	// choose a point on the plane 
	Eigen::Vector3f p0(projectedCloud.points[0].x, 
						projectedCloud.points[0].y, 
						projectedCloud.points[0].z); 
	
	for(unsigned int ii=0; ii<projectedCloud.points.size(); ii++) 
	{ 
		Eigen::Vector3f p3d(projectedCloud.points[ii].x, 
								projectedCloud.points[ii].y, 
								projectedCloud.points[ii].z); 

		// subtract all 3D points with a point in the plane 
		// this will move the origin of the 3D coordinate system 
		// onto the plane 
		p3d = p3d - p0; 

		cv::Point2f p2d; 
		p2d.x = p3d.dot(u); 
		p2d.y = p3d.dot(v); 
		points.push_back(p2d); 
	} 

	cv::Mat pointsMat(points); 
	cv::RotatedRect rrect = cv::minAreaRect(pointsMat); 
	cv::Point2f rrPts[4]; 
	rrect.points(rrPts); 

	//store the table top bounding points in a vector 
	for(unsigned int ii=0; ii<4; ii++) 
	{ 
		Eigen::Vector3f pbbx(rrPts[ii].x*u + rrPts[ii].y*v + p0); 
		corners.push_back(pbbx); 
	} 
	/*Eigen::Vector3f center(rrect.center.x*u + rrect.center.y*v + p0); 
	corners.push_back(center); */

	//Ver si se puede eliminar esto.
	vector<ofVec3f> vecCorners = projectPointsInPlane(corners,coefficients);

	//saveCloudAsFile("projectedrectangle.pcd",vecCorners);
	return vecCorners; 
} 

// -------------------------------------------------------------------------------------
// misc utils
// -------------------------------------------------------------------------------------

void computeBoundingBox(const PCPtr& cloud, PCXYZ& min, PCXYZ& max)
{
	ofVec3f vMin, vMax;
	computeBoundingBox(cloud, vMin, vMax);
	min = OFVEC3F_PCXYZ(vMin);
	max = OFVEC3F_PCXYZ(vMax);
}

void computeBoundingBox(const PCPtr& cloud, ofVec3f& vMin, ofVec3f& vMax)
{
	vMin = ofVec3f(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
	vMax = ofVec3f(-MAX_FLOAT, -MAX_FLOAT, -MAX_FLOAT);

	for (size_t k = 0; k < cloud->size(); k++) {
		PCXYZ p = cloud->at(k);
		vMin.x = min(p.x, vMin.x);
		vMin.y = min(p.y, vMin.y);
		vMin.z = min(p.z, vMin.z);
		vMax.x = max(p.x, vMax.x);
		vMax.y = max(p.y, vMax.y);
		vMax.z = max(p.z, vMax.z);
	}
}

PCPtr getHalo(const ofVec3f& min, const ofVec3f& max, const float& haloSize, const PCPtr& cloudSrc)
{
	PCPtr trimmedCloud (new PC());
	vector<int> indices;
	ofVec3f vMin = min - haloSize;
	ofVec3f vMax = max + haloSize;

	Eigen::Vector4f eMax(vMax.x,vMax.y,vMax.z,1);
	Eigen::Vector4f eMin(vMin.x,vMin.y,vMin.z,1);
	
	pcl::getPointsInBox(*cloudSrc,eMin,eMax,indices);
			
	for(int i = 0; i < indices.size(); i++)
		trimmedCloud->push_back(cloudSrc->at(indices.at(i)));
	return trimmedCloud;
}

ofVec3f computeCentroid(const PCPtr& cloud)
{
	Eigen::Vector4f eigenCentroid;
	pcl::compute3DCentroid(*cloud, eigenCentroid);
		
	return ofVec3f(eigenCentroid[0], eigenCentroid[1], eigenCentroid[2]);
}

int getDifferencesCloud(const PCPtr& src, 
						const PCPtr& tgt, 
						PCPtr& diff,
						float octreeRes)
{
	pcl::octree::OctreePointCloudChangeDetector<PCXYZ> octree (octreeRes);
	std::vector<int> newPointIdxVector;

	octree.setInputCloud(src);
	octree.addPointsFromInputCloud();
	octree.switchBuffers();
	octree.setInputCloud(tgt);
	octree.addPointsFromInputCloud();
	octree.getPointIndicesFromNewVoxels (newPointIdxVector);
	diff->points.reserve(newPointIdxVector.size());
	for (std::vector<int>::iterator it = newPointIdxVector.begin(); it != newPointIdxVector.end(); it++)
		diff->points.push_back(tgt->points[*it]);

	return newPointIdxVector.size();
}

int getDifferencesCount(const PCPtr& src, 
						const PCPtr& tgt, 
						float distanceThreshold)
{
	PCPtr small, big;
	if(src->size() < tgt->size())
	{
		small = src;
		big = tgt;
	}
	else
	{
		small = tgt;
		big = src;
	}

	//Seteo el kdtree con la segunda nube
	pcl::ExtractIndices<PCXYZ> extract;
	pcl::PointIndices::Ptr repeatedPoints (new pcl::PointIndices ());
	pcl::KdTreeFLANN<PCXYZ>::Ptr kdTree (new pcl::KdTreeFLANN<PCXYZ> (false));
	std::vector<int> kIndices (1);
	std::vector<float> kDistances (1);
	std::vector<int> indicesToRemove;
	int foundedPoints = 0;

	kdTree->setInputCloud(big);

	for (size_t i = 0; i < small->points.size (); ++i){
		foundedPoints = kdTree->nearestKSearch(small->at(i),1,kIndices,kDistances);
		if(foundedPoints > 0)
		{
			if(kDistances.at(0) < distanceThreshold)
				indicesToRemove.push_back(kIndices.at(0));
		}
	}

	return big->size() - indicesToRemove.size();
}

vector<pcl::PointIndices> findClusters(const PCPtr& cloud, float tolerance, int minClusterSize)
{
	return findClusters(cloud, tolerance, minClusterSize, cloud->size());
}

vector<pcl::PointIndices> findClusters(const PCPtr& cloud, float tolerance, int minClusterSize, int maxClusterSize)
{
	vector<pcl::PointIndices> result;
	if (cloud->size() >= minClusterSize)
	{
		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<PCXYZ>::Ptr tree(new pcl::search::KdTree<PCXYZ>);
		tree->setInputCloud(cloud);

		pcl::EuclideanClusterExtraction<PCXYZ> ec;
		ec.setClusterTolerance(tolerance); 
		ec.setMinClusterSize(minClusterSize);
		ec.setMaxClusterSize(maxClusterSize);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud);
		ec.extract(result);
	}
	
	return result;
}

vector<ofVec3f>::const_iterator findCloser(const ofVec3f& v, const vector<ofVec3f>& vertexs)
{
	float dist = numeric_limits<float>::max();
	vector<ofVec3f>::const_iterator idx = vertexs.begin();
	float curDist;
	for(vector<ofVec3f>::const_iterator iter = vertexs.begin(); iter != vertexs.end(); ++iter)
	{
		curDist = v.squareDistance(*iter);
		if(curDist < dist)
		{
			dist = curDist;
			idx = iter;
		}
	}
	return idx;

}

float computeVolume(const PCPtr& cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudHull (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud (cloud);
	chull.setDimension(3);
	chull.setComputeAreaVolume(true);
	chull.reconstruct (*cloudHull);

	//SOLO PARA FOTOS 
	saveCloud("cloudParaHull.pcd",*cloud);
	saveCloud("Hull.pcd",*cloudHull);
	////

	return chull.getTotalVolume();
}
// -------------------------------------------------------------------------------------