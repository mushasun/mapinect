#ifndef POTENTIALHAND_H__
#define POTENTIALHAND_H__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "ofxVec3f.h"
#include "utils.h"
#include <pcl/common/centroid.h>
using namespace pcl;

namespace mapinect {
	class PotentialHand {
	public:
		inline PotentialHand(PointCloud<PointXYZ>::Ptr	cloud, ofxVec3f centroid)
		{
			this->cloud = cloud;
			this->centroid = centroid;
			this->visited = false;
			this->timesVisited = 0;
		}

		inline PotentialHand(PointCloud<PointXYZ>::Ptr	cloud)
		{
			this->cloud = cloud;
			Eigen::Vector4f clusterCentroid;
			compute3DCentroid(*cloud,clusterCentroid);
			ofxVec3f ptoCentroid = ofxVec3f(clusterCentroid.x(),clusterCentroid.y(),clusterCentroid.z());
			this->centroid = ptoCentroid;
			this->visited = false;
			this->timesVisited = 0;
		}

		PointCloud<PointXYZ>::Ptr	cloud;
		bool						visited;
		int							timesVisited;
		ofxVec3f					centroid;
		
		inline bool operator==(const PotentialHand &rhs) { return (rhs.centroid - this->centroid).length() < 0.005; };
	};
}

#endif	// POTENTIALHAND