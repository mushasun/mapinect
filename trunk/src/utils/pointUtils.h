#ifndef POINT_UTILS_H__
#define POINT_UTILS_H__

#define PCD_EXTENSION		".pcd"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ofxVec3f.h"
#include <pcl/segmentation/segment_differences.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include "objectTypesEnum.h"
#include "PCPolyhedron.h"

#define DECLARE_POINTXYZ_OFXVEC3F(v, p)	ofxVec3f v = POINTXYZ_OFXVEC3F(p)
#define POINTXYZ_OFXVEC3F(p)			ofxVec3f(p.x, p.y, p.z)
#define PCLNORMAL_OFXVEC3F(p)			ofxVec3f(p.normal_x, p.normal_y, p.normal_z)
#define OFXVEC3F_POINTXYZ(v)			pcl::PointXYZ(v.x, v.y, v.z)

using namespace pcl;

void setPointXYZ(pcl::PointXYZ& p, float x, float y, float z);

void findPointCloudBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ& min, pcl::PointXYZ& max);
void findPointCloudBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, ofxVec3f& min, ofxVec3f& max);

//Obtiene el punto con menor Z
float getNearestPoint(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

//Devuelve la cantidad de diferencias entre la nuve cloud1 y cloud2 y devuelve las diferencias en diff
int getDifferencesCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, 
						pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2, 
						pcl::PointCloud<pcl::PointXYZ>::Ptr &diff,
						float distThreshold);

int getDifferencesCount(pcl::PointCloud<pcl::PointXYZ>::Ptr src, 
						pcl::PointCloud<pcl::PointXYZ>::Ptr tgt, 
						float distanceThreshold);


//Estima la normal utilizando un conjunto aleatorio de puntos, la cantidad tomada es especificada por plane->points.size() / NORMAL_ESTIMATION_PERCENT
ofxVec3f normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr plane);

//Estima la normal utilizando el conjunto de puntos especificado en indicesptr
ofxVec3f normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr plane, pcl::PointIndices::Ptr indicesptr);

PointCloud<PointXYZ>::Ptr getPartialCloudRealCoords(ofPoint min, ofPoint max, int density);

PointCloud<PointXYZ>::Ptr getCloud(int density);

PointCloud<PointXYZ>::Ptr getCloud();

pcl::PointIndices::Ptr adjustPlane(pcl::ModelCoefficients coefficients, PointCloud<pcl::PointXYZ>::Ptr cloudToAdjust);

float evaluatePoint(pcl::ModelCoefficients coefficients, ofxVec3f pto);

ObjectType getObjectType(pcl::PointCloud<pcl::PointXYZ>::Ptr src);

bool isInBorder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

bool onTable(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, mapinect::PCPolyhedron *table);
#endif // POINT_UTILS_H__
