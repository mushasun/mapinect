#include "pointUtils.h"
#include "utils.h"

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
