#include "DiscardCoordinate.h"

#include "ofVecUtils.h"

DiscardCoordinate calculateDiscardCoordinate(const ofVec3f& normal)
{
	ofVec3f absNormal = ofVec3f(fabsf(normal.x), fabsf(normal.y), fabsf(normal.z));
	if (absNormal.x > absNormal.y && absNormal.x > absNormal.z)
	{
		return kDiscardCoordinateX;
	}
	else if (absNormal.y > absNormal.z)
	{
		return kDiscardCoordinateY;
	}
	else
	{
		return kDiscardCoordinateZ;
	}
}

DiscardCoordinate calculateDiscardCoordinate(const vector<ofVec3f>& v)
{
	ofVec3f normal = computeNormal(v);
	return calculateDiscardCoordinate(normal);
}

ofVec2f discardCoordinateOfVec3f(const ofVec3f& v, DiscardCoordinate discard) {
	if (discard == kDiscardCoordinateX) {
		return ofVec2f(v.y, v.z);
	}
	else if (discard == kDiscardCoordinateY) {
		return ofVec2f(v.x, v.z);
	}
	else {	// kDiscardCoordinateZ
		return ofVec2f(v.x, v.y);
	}
}
