#ifndef MAPINECT_TRACKEDCLOUD_H__
#define MAPINECT_TRACKEDCLOUD_H__

#include <pcl/point_cloud.h>
#include "Object3D.h"

using namespace pcl;

namespace mapinect {
	class TrackedCloud {
	private:
		PointCloud<PointXYZ>::Ptr	cloud;
		int							counter;
		Object3D					*objectInModel;

	public:
		TrackedCloud(PointCloud<PointXYZ>::Ptr cloud);
		virtual ~TrackedCloud();

		bool matches(PointCloud<PointXYZ>::Ptr cloudCluster);
		void addCounter(int diff);
		inline int getCounter() { return counter; }
		inline bool hasObject() { return objectInModel != NULL; }

	};
}

#endif	// MAPINECT_TRACKEDCLOUD_H__
