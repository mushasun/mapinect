#ifndef POINT_UTILS_H__
#define POINT_UTILS_H__

#define PCD_EXTENSION		".pcd"

#include "mapinectTypes.h"
#include "ofPoint.h"
#include "objectTypesEnum.h"
#include "Polygon3D.h"

// -------------------------------------------------------------------------------------
// pcl <--> ofVec3f and other conversion utils
// -------------------------------------------------------------------------------------

#define PCXYZ_OFVEC3F(p)			ofVec3f(p.x, p.y, p.z)
#define PCLNORMAL_OFVEC3F(p)		ofVec3f(p.normal_x, p.normal_y, p.normal_z)
#define OFVEC3F_PCXYZ(v)			PCXYZ(v.x, v.y, v.z)

void setPCXYZ(PCXYZ& p, float x, float y, float z);

vector<ofVec3f>	pointCloudToOfVecVector(const PCPtr& cloud);
PCPtr			ofVecVectorToPointCloud(const vector<ofVec3f>& v);
vector<ofVec3f>	eigenVectorToOfVecVector(const vector<Eigen::Vector3f>& v);

PCPtr getCloudFromIndices(const PCPtr& cloud, const pcl::PointIndices& pi);

// -------------------------------------------------------------------------------------
// transformation utils
// -------------------------------------------------------------------------------------

PCXYZ	eyePos();
mapinect::Polygon3D transformPolygon3D(const mapinect::Polygon3D& polygon, const Eigen::Affine3f& transform);
mapinect::Polygon3D scalePolygon3D(const mapinect::Polygon3D& polygon, const float& scale);

PCXYZ	transformPoint(const PCXYZ& p, const Eigen::Affine3f& transform);
ofVec3f	transformPoint(const ofVec3f& v, const Eigen::Affine3f& transform);
PCPtr	transformCloud(const PCPtr& cloud, const Eigen::Affine3f& transform);

// maps points in world coordinates back to depth image coordinates
PCPtr			getScreenCoords(const PCPtr& transformedWorldCloud);
vector<ofVec3f>	getScreenCoords(const vector<ofVec3f>& transformedWorldCloud);
PCXYZ			getScreenCoords(const PCXYZ& transformedWorldPoint);
ofVec3f			getScreenCoords(const ofVec3f& transformedWorldPoint);

bool			isInViewField(ofVec3f vec);
// -------------------------------------------------------------------------------------
// i/o utils
// -------------------------------------------------------------------------------------

bool saveCloud(const string& filename, const PC& cloud);
void saveCloud(const string& filename, const ofVec3f& p);
void saveCloud(const string& filename, const vector<ofVec3f>& v);

PCPtr loadCloud(const string& filename);

// retrieves the depth from the device and creates a transformed point cloud
PCPtr getCloud(const ofPoint& min, const ofPoint& max, int stride);
PCPtr getCloud(int stride);
PCPtr getCloud();

PCPtr getCloudWithoutMutex(const ofPoint& min, const ofPoint& max, int stride);
PCPtr getCloudWithoutMutex(int stride);
PCPtr getCloudWithoutMutex();

// -------------------------------------------------------------------------------------
// plane utils
// -------------------------------------------------------------------------------------

ofVec3f getNormal(const pcl::ModelCoefficients&);

PCPtr extractBiggestPlane(const PCPtr& cloud, pcl::ModelCoefficients& coefficients, PCPtr& remainingCloud,
							float distanceThreshold, int maxIterations = 50);

pcl::PointIndices::Ptr adjustPlane(const pcl::ModelCoefficients& coefficients, const PCPtr& cloudToAdjust);

float evaluatePoint(const pcl::ModelCoefficients& coefficients, const PCXYZ& pto);
float evaluatePoint(const pcl::ModelCoefficients& coefficients, const ofVec3f& pto);

PCPtr projectPointsInPlane(const PCPtr& points, const pcl::ModelCoefficients& plane);
vector<ofVec3f> projectPointsInPlane(const vector<Eigen::Vector3f>& points, const pcl::ModelCoefficients& plane);

// -------------------------------------------------------------------------------------
// objects recognition utils
// -------------------------------------------------------------------------------------

ObjectType getObjectType(const PCPtr& src);

float boxProbability(const PCPtr& cloud);

vector<ofVec3f> findRectangle(const PCPtr& cloud, const pcl::ModelCoefficients& coefficients);

// -------------------------------------------------------------------------------------
// misc utils
// -------------------------------------------------------------------------------------

void computeBoundingBox(const PCPtr& cloud, PCXYZ& min, PCXYZ& max);
void computeBoundingBox(const PCPtr& cloud, ofVec3f& min, ofVec3f& max);
PCPtr getHalo(const ofVec3f& min, const ofVec3f& max, const float& haloSize, const PCPtr& cloudSrc);

ofVec3f computeCentroid(const PCPtr& cloud);

// Returns in diff the difference cloud between cloud1 and cloud2, and the count of different points found
int getDifferencesCloud(const PCPtr& cloud1, const PCPtr& cloud2, PCPtr &diff, float distThreshold);

int getDifferencesCount(const PCPtr& src, const PCPtr& tgt, float distanceThreshold);

vector<pcl::PointIndices> findClusters(const PCPtr& cloud, float tolerance, int minClusterSize);
vector<pcl::PointIndices> findClusters(const PCPtr& cloud, float tolerance, int minClusterSize, int maxClusterSize);

// -------------------------------------------------------------------------------------

#endif // POINT_UTILS_H__
