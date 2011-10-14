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