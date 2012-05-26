#include "ofVecUtils.h"

#include "utils.h"

ofVec3f BAD_OFVEC3F(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);

void findOfVec3fBoundingBox(const std::vector<ofVec3f>& v, ofVec3f &vMin, ofVec3f &vMax) {
	vMin = ofVec3f(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
	vMax = ofVec3f(-MAX_FLOAT, -MAX_FLOAT, -MAX_FLOAT);

	for (int k = 0; k < v.size(); k++) {
		ofVec3f p = v.at(k);
		vMin.x = minf(p.x, vMin.x);
		vMin.y = minf(p.y, vMin.y);
		vMin.z = minf(p.z, vMin.z);
		vMax.x = maxf(p.x, vMax.x);
		vMax.y = maxf(p.y, vMax.y);
		vMax.z = maxf(p.z, vMax.z);
	}

}

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
