#include "TrackedCloud.h"

#include "utils.h"

namespace mapinect {
	TrackedCloud::TrackedCloud(PointCloud<PointXYZ>::Ptr cloud) {
		this->cloud = cloud;
		counter = 2;
		objectInModel = NULL;
	}

	TrackedCloud::~TrackedCloud() {

	}

	void TrackedCloud::addCounter(int diff) {
		counter += diff;
		if (counter == 0) {
			if (hasObject()) {
				gModel3D->objectsMutex.lock();
					gModel3D->objects.remove(objectInModel);
					delete objectInModel;
					objectInModel = NULL;
				gModel3D->objectsMutex.unlock();
			}
		}
		else if(counter == TIMES_TO_CREATE_OBJ && !hasObject()) {
			counter = TIMES_TO_CREATE_OBJ + 2;
				
			gModel3D->objectsMutex.lock();
				objectInModel = new Object3D(cloud, cloud);
				gModel3D->objects.push_back(objectInModel);
			gModel3D->objectsMutex.unlock();
		}
	}

	bool TrackedCloud::matches(PointCloud<PointXYZ>::Ptr cloud_cluster) {
		PointCloud<PointXYZ>::Ptr difCloud (new PointCloud<PointXYZ>);
		int dif = getDifferencesCloud(cloud, cloud_cluster, difCloud, OCTREE_RES);
		if(dif < DIFF_IN_OBJ)
		{
			return true;
		}
		return false;
	}
}
