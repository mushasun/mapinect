#ifndef MAPINECT_DISCARDCOORDINATE_H__
#define MAPINECT_DISCARDCOORDINATE_H__

#include "ofVec3f.h"
#include "ofVec2f.h"

enum DiscardCoordinate
{
	kDiscardCoordinateX,
	kDiscardCoordinateY,
	kDiscardCoordinateZ
};

DiscardCoordinate calculateDiscardCoordinate(const ofVec3f& normal);
DiscardCoordinate calculateDiscardCoordinate(const vector<ofVec3f>& v);
ofVec2f discardCoordinateOfVec3f(const ofVec3f& v, DiscardCoordinate discard);

#endif	// MAPINECT_DISCARDCOORDINATE_H__