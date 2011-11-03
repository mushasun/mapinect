#include "ofxVecUtils.h"

void findOfxVec3fBoundingBox(const std::vector<ofxVec3f>& v, ofxVec3f &vMin, ofxVec3f &vMax) {
	vMin = ofxVec3f(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
	vMax = ofxVec3f(-MAX_FLOAT, -MAX_FLOAT, -MAX_FLOAT);

	for (int k = 0; k < v.size(); k++) {
		ofxVec3f p = v.at(k);
		if (p.x < vMin.x) {
			vMin.x = p.x;
		}
		if (p.x > vMax.x) {
			vMax.x = p.x;
		}
		if (p.y < vMin.y) {
			vMin.y = p.y;
		}
		if (p.y > vMax.y) {
			vMax.y = p.y;
		}
		if (p.z < vMin.z) {
			vMin.z = p.z;
		}
		if (p.z > vMax.z) {
			vMax.z = p.z;
		}
	}

}

DiscardCoordinate calculateDiscardCoordinate(const ofxVec3f& v) {
	if (v.x <= v.y && v.x <= v.z) {
		return kDiscardCoordinateX;
	}
	else if (v.y <= v.x && v.y <= v.z) {
		return kDiscardCoordinateY;
	}
	else {
		return kDiscardCoordinateZ;
	}
}

DiscardCoordinate calculateDiscardCoordinate(const ofxVec3f& min, const ofxVec3f& max) {
	ofxVec3f dif = max - min;
	return calculateDiscardCoordinate(dif);
}

ofxVec2f discardCoordinateOfxVec3f(const ofxVec3f& v, DiscardCoordinate discard) {
	if (discard == kDiscardCoordinateX) {
		return ofxVec2f(v.y, v.z);
	}
	else if (discard == kDiscardCoordinateY) {
		return ofxVec2f(v.x, v.z);
	}
	else {	// kDiscardCoordinateZ
		return ofxVec2f(v.x, v.y);
	}
}
