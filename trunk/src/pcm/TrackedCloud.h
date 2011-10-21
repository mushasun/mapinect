#ifndef MAPINECT_TRACKED_CLOUD_H__
#define MAPINECT_TRACKED_CLOUD_H__

#include "PCModelObject.h"
#include "pointUtils.h"

using namespace pcl;

namespace mapinect {
	class TrackedCloud {
		public:
			TrackedCloud() { }
			TrackedCloud(PointCloud<PointXYZ>::Ptr cloud);
			virtual ~TrackedCloud();

			bool matches(PointCloud<PointXYZ>::Ptr cloudCluster);
			void addCounter(int diff);
			inline int getCounter() { return counter; }
			inline bool hasObject() { return objectInModel != NULL; }

		private:
			PointCloud<PointXYZ>::Ptr	cloud;
			int							counter;
			PCModelObject				*objectInModel;

	};
}

#endif	// MAPINECT_TRACKED_CLOUD_H__
