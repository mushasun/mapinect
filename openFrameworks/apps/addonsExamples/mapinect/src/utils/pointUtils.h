#ifndef POINT_UTILS_H__
#define POINT_UTILS_H__

#define PCD_EXTENSION		".pcd"

#include "mapinectTypes.h"
#include "ofPoint.h"
#include "objectTypesEnum.h"

#define DECLARE_POINTXYZ_OFXVEC3F(v, p)	ofVec3f v = POINTXYZ_OFXVEC3F(p)
#define POINTXYZ_OFXVEC3F(p)			ofVec3f(p.x, p.y, p.z)
#define PCLNORMAL_OFXVEC3F(p)			ofVec3f(p.normal_x, p.normal_y, p.normal_z)
#define OFXVEC3F_POINTXYZ(v)			pcl::PointXYZ(v.x, v.y, v.z)

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
ofVec3f normalEstimation(const PCPtr& plane, const pcl::PointIndices::Ptr& indices);

PCPtr getPartialCloudRealCoords(const ofPoint& min, const ofPoint& max, int density);

PCPtr getCloud(int density);

PCPtr getCloud();

pcl::PointIndices::Ptr adjustPlane(const pcl::ModelCoefficients& coefficients, const PCPtr& cloudToAdjust);

ofVec3f computeCentroid(const PCPtr& cloud);

float evaluatePoint(const pcl::ModelCoefficients& coefficients, const ofVec3f& pto);

ObjectType getObjectType(const PCPtr& src);

bool isInBorder(const PCPtr& cloud);

void createCloud(const ofVec3f& pto, const string& name);

void createCloud(const vector<ofVec3f>& ptos, const string& name);

bool saveCloudAsFile(const std::string &file_name, const PC &cloud);


bool xAxisSortAsc (ofVec3f i,ofVec3f j);
bool xAxisSortDes (ofVec3f i,ofVec3f j);
bool yAxisSortAsc (ofVec3f i,ofVec3f j);
bool yAxisSortDes (ofVec3f i,ofVec3f j);
pcl::ModelCoefficients findPlane(ofVec3f v1, ofVec3f v2, ofVec3f v3);

ofVec3f planeIntersection(const pcl::ModelCoefficients& p1, const pcl::ModelCoefficients& p2, const pcl::ModelCoefficients& p3);
#endif // POINT_UTILS_H__
