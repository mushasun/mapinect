#include "ofVecUtils.h"

void findOfxVec3fBoundingBox(const std::vector<ofVec3f>& v, ofVec3f &vMin, ofVec3f &vMax) {
	vMin = ofVec3f(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
	vMax = ofVec3f(-MAX_FLOAT, -MAX_FLOAT, -MAX_FLOAT);

	for (int k = 0; k < v.size(); k++) {
		ofVec3f p = v.at(k);
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

DiscardCoordinate calculateDiscardCoordinate(const ofVec3f& v) {
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

DiscardCoordinate calculateDiscardCoordinate(const ofVec3f& min, const ofVec3f& max) {
	ofVec3f dif = max - min;
	return calculateDiscardCoordinate(dif);
}

ofVec2f discardCoordinateOfxVec3f(const ofVec3f& v, DiscardCoordinate discard) {
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

