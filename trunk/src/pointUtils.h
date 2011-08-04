#ifndef POINT_UTILS_H__
#define POINT_UTILS_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define DECLARE_POINTXYZ_OFXVEC3F(v, p)	ofxVec3f v = POINTXYZ_OFXVEC3F(p)
#define POINTXYZ_OFXVEC3F(p)			ofxVec3f(p.x, p.y, p.z)
#define OFXVEC3F_POINTXYZ(v)			pcl::PointXYZ(v.x, v.y, v.z)

void setPointXYZ(pcl::PointXYZ& p, float x, float y, float z);

void findPointCloudBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ& min, pcl::PointXYZ& max);

#endif // POINT_UTILS_H__
