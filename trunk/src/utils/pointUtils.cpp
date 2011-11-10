#include "pointUtils.h"
#include "utils.h"
#include "pcl/octree/octree.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

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
