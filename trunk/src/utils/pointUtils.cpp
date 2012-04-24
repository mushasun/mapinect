#include "pointUtils.h"
#include "utils.h"
#include "Line2D.h"
#include "pcl/octree/octree.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include "PCQuadrilateral.h"
#include "PCPolyhedron.h"
#include "PCPolygon.h"
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/filters/passthrough.h>
#include "ofxVec2f.h"
#include "HandDetector.h"

void setPointXYZ(pcl::PointXYZ& p, float x, float y, float z) {
	p.x = x;
	p.y = y;
	p.z = z;
}


void findPointCloudBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ& min, pcl::PointXYZ& max) {
	setPointXYZ(min, MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
	setPointXYZ(max, -MAX_FLOAT, -MAX_FLOAT, -MAX_FLOAT);

	for (size_t k = 0; k < cloud->size(); k++) {
		pcl::PointXYZ p = cloud->at(k);
		if (p.x < min.x) {
			min.x = p.x;
		}
		if (p.x > max.x) {
			max.x = p.x;
		}
		if (p.y < min.y) {
			min.y = p.y;
		}
		if (p.y > max.y) {
			max.y = p.y;
		}
		if (p.z < min.z) {
			min.z = p.z;
		}
		if (p.z > max.z) {
			max.z = p.z;
		}
	}

}

void findPointCloudBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ofxVec3f& min, ofxVec3f& max) {
	min = ofxVec3f(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
	max = ofxVec3f(-MAX_FLOAT, -MAX_FLOAT, -MAX_FLOAT);

	for (size_t k = 0; k < cloud->size(); k++) {
		pcl::PointXYZ p = cloud->at(k);
		if (p.x < min.x) {
			min.x = p.x;
		}
		if (p.x > max.x) {
			max.x = p.x;
		}
		if (p.y < min.y) {
			min.y = p.y;
		}
		if (p.y > max.y) {
			max.y = p.y;
		}
		if (p.z < min.z) {
			min.z = p.z;
		}
		if (p.z > max.z) {
			max.z = p.z;
		}
	}

}


float getNearestPoint(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
	int size = cloud->size();
	float closest = MAX_FLOAT;
	for(register int i = 0; i < size; i++){
		float current = cloud->at(i).z;
		if(current < closest && current > 0)
			closest = cloud->at(i).z;
	}
	return closest;
}

int getDifferencesCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr src, 
						pcl::PointCloud<pcl::PointXYZ>::ConstPtr tgt, 
						pcl::PointCloud<pcl::PointXYZ>::Ptr &diff,
						float octreeRes)
{
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (octreeRes);
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

	////Seteo el kdtree con la segunda nube
	//pcl::ExtractIndices<pcl::PointXYZ> extract;
	//pcl::PointIndices::Ptr oldPoints (new pcl::PointIndices ());
	//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdTree (new pcl::KdTreeFLANN<pcl::PointXYZ> (false));
	//std::vector<int> k_indices (1);
	//std::vector<float> k_distances (1);
	//std::vector<int> indicesToRemove;
	//int foundedPoints = 0;

	//kdTree->setInputCloud(tgt);

	//for (size_t i = 0; i < src->points.size (); ++i){
	//	foundedPoints = kdTree->nearestKSearch(src->at(i),1,k_indices,k_distances);
	//	if(foundedPoints == 1)
	//	{
	//		if(k_distances.at(0) < distThreshold)
	//			indicesToRemove.push_back(k_indices.at(0));
	//	}
	//}

	////quito los puntos de tgt que ya están en src
	//
	//oldPoints->indices = indicesToRemove;
	//extract.setInputCloud (tgt);
	//extract.setIndices (oldPoints);
	//extract.setNegative (true);
	//
	////FIX
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_temp (new pcl::PointCloud<pcl::PointXYZ>());
	//if(tgt->size() != indicesToRemove.size())
	//	extract.filter (*cloud_filtered_temp);
	//	
	//diff = cloud_filtered_temp;

	//return diff->size();
}

int getDifferencesCount(pcl::PointCloud<pcl::PointXYZ>::Ptr src, 
						pcl::PointCloud<pcl::PointXYZ>::Ptr tgt, 
						float distanceThreshold)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr small, big;
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
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointIndices::Ptr repeatedPoints (new pcl::PointIndices ());
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdTree (new pcl::KdTreeFLANN<pcl::PointXYZ> (false));
	std::vector<int> k_indices (1);
	std::vector<float> k_distances (1);
	std::vector<int> indicesToRemove;
	int foundedPoints = 0;

	kdTree->setInputCloud(big);

	for (size_t i = 0; i < small->points.size (); ++i){
		foundedPoints = kdTree->nearestKSearch(small->at(i),1,k_indices,k_distances);
		if(foundedPoints > 0)
		{
			if(k_distances.at(0) < distanceThreshold)
				indicesToRemove.push_back(k_indices.at(0));
		}
	}

	return big->size() - indicesToRemove.size();
}

ofxVec3f normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr plane)
{
	//Calculo de normales
		//Random indices
	int sampleSize = floor (plane->points.size() * NORMAL_ESTIMATION_PERCENT);
	std::vector<int> indices (sampleSize);
	for (size_t i = 0; i < sampleSize; i++) 
		indices[i] = rand() % plane->points.size();

	pcl::PointIndices::Ptr indicesptr (new pcl::PointIndices ());
	indicesptr->indices = indices;


	return normalEstimation(plane, indicesptr);
}

ofxVec3f normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr plane, pcl::PointIndices::Ptr indicesptr)
{
	//Calculo de normales
	int sampleSize = indicesptr->indices.size();
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (plane);
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	
	ne.setIndices(indicesptr);
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.3);

	// Compute the features
	ne.compute (*cloud_normals);

	ofxVec3f result(0,0,0);
	//Promedio las normales
	float bad_point = std::numeric_limits<float>::quiet_NaN();
	for(int i = 0; i < sampleSize ; i++){
		pcl::Normal normal = cloud_normals->at(i);
		if (normal.normal_z != bad_point && normal.normal_z == normal.normal_z)
			result += PCLNORMAL_OFXVEC3F(cloud_normals->at(i));
	}
	
	result /= sampleSize;
	result = result.normalize();
	return result;
}

PointCloud<PointXYZ>::Ptr getPartialCloudRealCoords(ofPoint min, ofPoint max, int density){
		//Chequeo
		if (min.x < 0 || min.y < 0 || max.x > KINECT_WIDTH || max.y > KINECT_HEIGHT || min.x > max.x || min.y > max.y) //* load the file
		{
			PCL_ERROR ("Error en parametros de entrada para obtener la nube \n");
		}

		//Calcular tamaño de la nube
		PointCloud<PointXYZ>::Ptr partialColud (new pcl::PointCloud<pcl::PointXYZ>);
		partialColud->width    = ceil((max.x - min.x)/density);
		partialColud->height   = ceil((max.y - min.y)/density);
		partialColud->is_dense = false;
		partialColud->points.resize (partialColud->width*partialColud->height);
		register float* depth_map = gKinect->getDistancePixels();
		//Recorrer el mapa de distancias obteniendo sólo los que estén dentro del rectángulo
		register int depth_idx = 0;
		int cloud_idx = 0;
		for(int v = min.y; v < max.y; v += density) {
			for(register int u = min.x; u < max.x; u += density) {
				pcl::PointXYZ& pt = partialColud->points[cloud_idx];
				cloud_idx++;

				// Check for invalid measurements
				if(depth_map[depth_idx] == 0){
					pt.x = pt.y = pt.z = 0;
				}
				else
				{
					ofxVec3f pto = gKinect->getWorldCoordinateFor(u,v);

					if(pto.z > MAX_Z)
						pt.x = pt.y = pt.z = 0;
					else
					{
						pt.x = pto.x;
						pt.y = pto.y;
						pt.z = pto.z;
					}
				}

				depth_idx += density;
			}
			//pcl::io::savePCDFileASCII ("partial_real.pcd", *partialColud);
		}
		return partialColud;	

	}

PointCloud<PointXYZ>::Ptr getCloud(int density){
		return getPartialCloudRealCoords(ofPoint(KINECT_WIDTH_OFFSET,KINECT_HEIGHT_OFFSET),ofPoint(KINECT_WIDTH,KINECT_HEIGHT),density);
	}

PointCloud<PointXYZ>::Ptr getCloud(){
	return getPartialCloudRealCoords(ofPoint(KINECT_WIDTH_OFFSET,KINECT_HEIGHT_OFFSET),ofPoint(KINECT_WIDTH,KINECT_HEIGHT),CLOUD_RES);
}

PointIndices::Ptr adjustPlane(ModelCoefficients coefficients, PointCloud<PointXYZ>::Ptr cloudToAdjust)
{
	float PLANE_THRESHOLD = 0.009;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	for (int k = 0; k < cloudToAdjust->size(); k++) {
		PointXYZ pto = cloudToAdjust->at(k);
		float val = coefficients.values[0] * pto.x + 
					coefficients.values[1] * pto.y + 
					coefficients.values[2] * pto.z + 
					coefficients.values[3]; 

		if(abs(val) < PLANE_THRESHOLD)
			inliers->indices.push_back(k);
	}
	return inliers;
}

float evaluatePoint(pcl::ModelCoefficients coefficients, ofxVec3f pto)
{
	return coefficients.values.at(0) * pto.x +
		   coefficients.values.at(1) * pto.y +
		   coefficients.values.at(2) * pto.z +
		   coefficients.values.at(3);
}

float boxProbability(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,mapinect::PCPolygon* table)
{
	if(onTable(cloud,table))
	{
		vector<ofxVec3f> normals;
		float totalPoints = cloud->size();
		float pointsInPlanes = 0;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
		PointCloud<pcl::PointXYZ>::Ptr cloudTemp (new PointCloud<PointXYZ>(*cloud));
	
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (50);
		seg.setDistanceThreshold (0.009); //original: 0.01

		// Create the filtering object
		int i = 0, nr_points = cloud->points.size ();

		int numFaces = 0;
		while (cloudTemp->points.size () > 0.07 * nr_points && numFaces < 8)
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
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_temp_inliers (new pcl::PointCloud<pcl::PointXYZ>());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_temp_outliers (new pcl::PointCloud<pcl::PointXYZ>());
			if(inliers->indices.size() != cloudTemp->size())
			{
				// Extract the inliers
				extract.setInputCloud (cloudTemp);
				extract.setIndices (inliers);
				extract.setNegative (false);
				extract.filter (*cloud_filtered_temp_inliers);
				cloud_p = cloud_filtered_temp_inliers;
			}
			else
				cloud_p = cloudTemp;
		
			ofxVec3f norm = normalEstimation(cloud_p);

			//Chequeo que las normales sean perpendiculares entre si y paraleas o perpendiculares a la mesa.
			if(table != NULL)
			{
				ofxVec3f tableNormal = table->getNormal();

				float dot = abs(tableNormal.dot(norm));
				if( dot > 0.1 && dot < 0.9)
					return 0;
				//si es paralela a la mesa, chequeo que esté sobre la mesa
				if(dot > 0.9)
					if(!onTable(cloud_p,table))
						return 0;
			}
			for(int i = 0; i < normals.size(); i++)
			{
				float dot = abs(normals[i].dot(norm));
				if( dot > 0.1)
					return 0;
			}

			normals.push_back(norm);
			// Create the filtering object
			extract.setInputCloud (cloudTemp);
			extract.setIndices (inliers);
			extract.setNegative (true);

		
			if(cloud_p->size() != cloudTemp->size())
				extract.filter (*cloud_filtered_temp_outliers);

			cloudTemp = cloud_filtered_temp_outliers;

			i++;
			numFaces++;
		}

		if(numFaces > 0 && numFaces < 4)
			return pointsInPlanes/totalPoints;
		else
			return 0;
	}
	else
		return 0;
}

int tmpFingerCount = 0;
bool isFingerTip(pcl::octree::OctreePointCloud<pcl::PointXYZ>::Ptr ot, ofxVec3f potential_finger_tip)
{
	pcl::PointXYZ pto_left(potential_finger_tip.x - 0.02, potential_finger_tip.y,0);
	pcl::PointXYZ pto_right(potential_finger_tip.x + 0.02, potential_finger_tip.y,0);
	
	bool neighbour_left = ot->isVoxelOccupiedAtPoint(pto_left);
	bool neighbour_right = ot->isVoxelOccupiedAtPoint(pto_right);

	if(!neighbour_left && !neighbour_right)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp2_max (new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointXYZ pt = OFXVEC3F_POINTXYZ(potential_finger_tip);
		cloud_tmp2_max->points.push_back(pt);
		//pcl::io::savePCDFileASCII ("fingertip" + ofToString(++tmpFingerCount) + ".pcd", *cloud_tmp2_max);
		
		return true;
	}
	else
		return false;
}

bool isInFingers(vector<mapinect::Line2D> fingers, ofxVec3f pto)
{
	for(int i = 0; i < fingers.size(); i++)
	{
		if(fingers.at(i).distance(ofxVec2f(pto.x,pto.y)) < 0.02)
			return true;
	}
	
	return false;
}

float handProbability(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, mapinect::PCPolygon* table)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr clusterBack (new pcl::PointCloud<pcl::PointXYZ>(*cloud));
	mapinect::HandDetector hd;
	hd.SetPotentialHandCloud(clusterBack);
	hd.SetTable(table);
	return hd.IsHand();
}

mapinect::PCPolygon* getTable()
{
	mapinect::PCPolygon* table;
	mapinect::PCPolyhedron* tablePol = dynamic_cast<mapinect::PCPolyhedron*>(gModel->table);
	if (tablePol->getPCPolygonSize() > 0) {
		table = tablePol->getPCPolygon(0);
	}

	return table;
}

ObjectType getObjectType(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	mapinect::PCPolygon* table = getTable();

	float box_prob = boxProbability(cloud, table);

	float hand_prob = handProbability(cloud,table);

	ObjectType ret;
	
	float max_prob = max(hand_prob,box_prob);

	if(max_prob > .80)
	{
		if(hand_prob == max_prob)
			return ret = HAND;
		else
			return ret = BOX;
	}
	else
		return ret = UNRECOGNIZED;
}

//Debería tener en cuenta el Z para saber si está en el borde
bool isInBorder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cliped_cloud (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-1, -0.35);
	pass.filter (*cliped_cloud);

	if(!cliped_cloud->empty())
		return true;
	else
	{
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (0.35, 1);
		pass.filter (*cliped_cloud);
		if(!cliped_cloud->empty())
			return true;
		else
		{
			pass.setFilterFieldName ("y");
			pass.setFilterLimits (-1, -0.25);
			pass.filter (*cliped_cloud);

			if(!cliped_cloud->empty())
				return true;
		}
	}

	return false;
}
//Para debug
//Crea una nube a partir del punto y nombre por parámetro
void createCloud(ofxVec3f pto, string name)
{
	PCDWriter writer;
	PointXYZ ptoXYZ = PointXYZ(pto.x,pto.y,pto.z);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->push_back(ptoXYZ);
	writer.write<pcl::PointXYZ> (name, *cloud, false);
}



//True si está en contacto con la mesa

bool onTable(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, mapinect::PCPolygon *table)
{
	//Busco el mayor y
	if(table != NULL)
	{
		int idx_max = -1;
		float max = -1;
		for(int i = 0; i < cloud->points.size(); i++)
		{
			if(cloud->points.at(i).y > max)
			{
				idx_max = i;
				max = cloud->points.at(i).y;
			}
		}

		return abs(evaluatePoint(table->getCoefficients(),POINTXYZ_OFXVEC3F(cloud->points.at(idx_max)))) < 0.03;
	}
	else
		return false;
}

//True si está en contacto con la mesa
bool onTable(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, mapinect::PCPolyhedron *table)
{
	return onTable(cloud,table->getPCPolygon(0));
}

float minf(float x, float y)
{
	if(x <= y)
		return x;
	return y;
}

float maxf(float x, float y)
{
	if(x >= y)
		return x;
	return y;
}

PointCloud<PointXYZRGB>::Ptr getPartialColorCloudRealCoords(ofPoint min, ofPoint max, int density){
		//Chequeo
		if (min.x < 0 || min.y < 0 || max.x > KINECT_WIDTH || max.y > KINECT_HEIGHT || min.x > max.x || min.y > max.y) //* load the file
		{
			PCL_ERROR ("Error en parametros de entrada para obtener la nube \n");
		}

		//Calcular tamaño de la nube
		PointCloud<PointXYZRGB>::Ptr partialColud (new pcl::PointCloud<pcl::PointXYZRGB>);
		partialColud->width    = ceil((max.x - min.x)/density);
		partialColud->height   = ceil((max.y - min.y)/density);
		partialColud->is_dense = false;
		partialColud->points.resize (partialColud->width*partialColud->height);
		register float* depth_map = gKinect->getDistancePixels();
		//Recorrer el mapa de distancias obteniendo sólo los que estén dentro del rectángulo
		register int depth_idx = 0;
		int cloud_idx = 0;
		for(int v = min.y; v < max.y; v += density) {
			for(register int u = min.x; u < max.x; u += density) {
				pcl::PointXYZRGB& pt = partialColud->points[cloud_idx];
				cloud_idx++;

				// Check for invalid measurements
				if(depth_map[depth_idx] == 0){
					pt.x = pt.y = pt.z = 0;
				}
				else
				{
					ofxVec3f pto = gKinect->getWorldCoordinateFor(u,v);
					ofColor color = gKinect->getCalibratedColorAt(ofPoint(u,v));

					if(pto.z > MAX_Z)
						pt.x = pt.y = pt.z = 0;
					else
					{
						pt.x = pto.x;
						pt.y = pto.y;
						pt.z = pto.z;

						uint8_t r = color.r; 

						uint8_t g = color.g; 

						uint8_t b = color.b;           

						//cout<<v,<<r << "," << g << "," << b <<endl;
						uint32_t rgb_val = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

						pt.rgb = *reinterpret_cast<float*>(&rgb_val);
					}
				}

				depth_idx += density;
			}
		}
		return partialColud;	

	}

bool tableParallel(mapinect::PCPolygon *polygon, mapinect::PCPolygon *table)
{
	ofxVec3f tableNormal = table->getNormal();
	ofxVec3f polygonNormal = polygon->getNormal();
	float dot = abs(tableNormal.dot(polygonNormal));
	return dot > 0.9;
}
