#ifndef OFXVEC_UTILS_H__
#define OFXVEC_UTILS_H__

#include "ofxVec3f.h"
#include "ofxVec2f.h"
#include "utils.h"

void findOfxVec3fBoundingBox(const std::vector<ofxVec3f>& v, ofxVec3f &vMin, ofxVec3f &vMax);


enum DiscardCoordinate {
	kDiscardCoordinateX,
	kDiscardCoordinateY,
	kDiscardCoordinateZ
};

DiscardCoordinate calculateDiscardCoordinate(const ofxVec3f& v);
DiscardCoordinate calculateDiscardCoordinate(const ofxVec3f& min, const ofxVec3f& max);
ofxVec2f discardCoordinateOfxVec3f(const ofxVec3f& v, DiscardCoordinate discard);

#endif	// OFXVEC_UTILS_H__