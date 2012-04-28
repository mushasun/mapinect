#ifndef POTENTIALHAND_H__
#define POTENTIALHAND_H__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "ofVec3f.h"
#include "utils.h"
#include <pcl/common/centroid.h>
using namespace pcl;

namespace mapinect {
	class PotentialHand {
	public:
		PotentialHand(PointCloud<PointXYZ>::Ptr	cloud, ofVec3f centroid);
		PotentialHand(PointCloud<PointXYZ>::Ptr	cloud);

		PointCloud<PointXYZ>::Ptr	cloud;
		bool						visited;
		int							timesVisited;
		ofVec3f					centroid;
		
		inline bool operator==(const PotentialHand &rhs) { return (rhs.centroid - this->centroid).length() < 0.005; };
	};
}

#endif	// POTENTIALHAND