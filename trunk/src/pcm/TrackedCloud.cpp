#include "TrackedCloud.h"

#include "utils.h"
#include "PCPolyhedron.h"

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
				gModel->objectsMutex.lock();
					gModel->objects.remove(objectInModel);
					delete objectInModel;
					objectInModel = NULL;
				gModel->objectsMutex.unlock();
			}
		}
		else if(counter == TIMES_TO_CREATE_OBJ && !hasObject()) {
			counter = TIMES_TO_CREATE_OBJ + 2;
				
			gModel->objectsMutex.lock();
				objectInModel = new PCPolyhedron(cloud, cloud);
				objectInModel->detectPrimitives();
				gModel->objects.push_back(objectInModel);
			gModel->objectsMutex.unlock();
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
