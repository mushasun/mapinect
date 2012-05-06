#ifndef POINT_UTILS_H__
#define POINT_UTILS_H__

#define PCD_EXTENSION		".pcd"

#include "mapinectTypes.h"
#include "ofVec3f.h"
#include <pcl/segmentation/segment_differences.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include "objectTypesEnum.h"
#include "PCPolyhedron.h"

#define DECLARE_POINTXYZ_OFXVEC3F(v, p)	ofVec3f v = POINTXYZ_OFXVEC3F(p)
#define POINTXYZ_OFXVEC3F(p)			ofVec3f(p.x, p.y, p.z)
#define PCLNORMAL_OFXVEC3F(p)			ofVec3f(p.normal_x, p.normal_y, p.normal_z)
#define OFXVEC3F_POINTXYZ(v)			pcl::PointXYZ(v.x, v.y, v.z)

using namespace pcl;

void setPointXYZ(pcl::PointXYZ& p, float x, float y, float z);

vector<ofVec3f> pointCloudToOfVecVector(const PCPtr& cloud);

void findPointCloudBoundingBox(const PCPtr& cloud, pcl::PointXYZ& min, pcl::PointXYZ& max);
void findPointCloudBoundingBox(const PCPtr& cloud, ofVec3f& min, ofVec3f& max);

//Obtiene el punto con menor Z
float getNearestPoint(const PCPtr& cloud);

//Devuelve la cantidad de diferencias entre la nuve cloud1 y cloud2 y devuelve las diferencias en diff
int getDifferencesCloud(const PCPtr& cloud1, 
						const PCPtr& cloud2, 
						PCPtr &diff,
						float distThreshold);

int getDifferencesCount(const PCPtr& src, 
						const PCPtr& tgt, 
						float distanceThreshold);


//Estima la normal utilizando un conjunto aleatorio de puntos, la cantidad tomada es especificada por plane->points.size() / NORMAL_ESTIMATION_PERCENT
ofVec3f normalEstimation(const PCPtr& plane);

//Estima la normal utilizando el conjunto de puntos especificado en indicesptr
ofVec3f normalEstimation(const PCPtr& plane, pcl::PointIndices::Ptr indicesptr);

PCPtr getPartialCloudRealCoords(ofPoint min, ofPoint max, int density);

PCPtr getCloud(int density);

PCPtr getCloud();

pcl::PointIndices::Ptr adjustPlane(const pcl::ModelCoefficients& coefficients, const PCPtr& cloudToAdjust);

float evaluatePoint(const pcl::ModelCoefficients& coefficients, ofVec3f pto);

ObjectType getObjectType(const PCPtr& src);

bool isInBorder(const PCPtr& cloud);

bool onTable(const PCPtr& cloud, mapinect::Table *table);

bool tableParallel(mapinect::PCPolygon *polygon, mapinect::Table *table);

void createCloud(ofVec3f pto, string name);

float maxf(float x, float y);

float minf(float x, float y);

PointCloud<PointXYZRGB>::Ptr getPartialColorCloudRealCoords(ofPoint min, ofPoint max, int density);
#endif // POINT_UTILS_H__
