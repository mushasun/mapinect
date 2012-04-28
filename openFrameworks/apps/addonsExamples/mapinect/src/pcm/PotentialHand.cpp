#include "PotentialHand.h"

#include <pcl/common/centroid.h>
using namespace pcl;

namespace mapinect {
	PotentialHand::PotentialHand(PointCloud<PointXYZ>::Ptr	cloud, ofxVec3f centroid)
	{
		this->cloud = cloud;
		this->centroid = centroid;
		this->visited = false;
		this->timesVisited = 0;
	}

	PotentialHand::PotentialHand(PointCloud<PointXYZ>::Ptr	cloud)
	{
		this->cloud = cloud;
		Eigen::Vector4f clusterCentroid;
		compute3DCentroid(*cloud,clusterCentroid);
		ofxVec3f ptoCentroid = ofxVec3f(clusterCentroid.x(),clusterCentroid.y(),clusterCentroid.z());
		this->centroid = ptoCentroid;
		this->visited = false;
		this->timesVisited = 0;
	}
}

