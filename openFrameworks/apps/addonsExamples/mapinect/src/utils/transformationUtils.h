#ifndef TRANSFORMATION_UTILS_H__
#define TRANSFORMATION_UTILS_H__

#include "mapinectTypes.h"
#include "Polygon3D.h"
#include "ofVec3f.h"

using namespace mapinect;

#define PCXYZ_EIGEN3F(v)	Eigen::Vector3f(v.x, v.y, v.z)
#define OFVEC3F_EIGEN3F(v)	Eigen::Vector3f(v.x, v.y, v.z)

vector<ofVec3f>				eigenVectorToOfVecVector(const vector<Eigen::Vector3f>& v);

Eigen::Affine3f				getTranslationMatrix(const ofVec3f& v);
Eigen::Affine3f				getScaleMatrix(const ofVec3f& v);
Eigen::Affine3f				getScaleMatrix(float scale);
Eigen::Affine3f				getRotationMatrix(const ofVec3f& axis, float angleRad);

PCXYZ						transformPoint(const PCXYZ& p, const Eigen::Affine3f& transform);
ofVec3f						transformPoint(const ofVec3f& v, const Eigen::Affine3f& transform);
PCPtr						transformCloud(const PCPtr& cloud, const Eigen::Affine3f& transform);
vector<ofVec3f>				transformVector(const vector<ofVec3f>& v, const Eigen::Affine3f& transform);
Polygon3D					transformPolygon3D(const Polygon3D& polygon, const Eigen::Affine3f& transform);

PCXYZ						eyePos();

// maps points in world coordinates back to depth image coordinates
PCPtr						getScreenCoords(const PCPtr& transformedWorldCloud);
vector<ofVec3f>				getScreenCoords(const vector<ofVec3f>& transformedWorldCloud);
PCXYZ						getScreenCoords(const PCXYZ& transformedWorldPoint);
ofVec3f						getScreenCoords(const ofVec3f& transformedWorldPoint);

#endif // TRANSFORMATION_UTILS_H__
