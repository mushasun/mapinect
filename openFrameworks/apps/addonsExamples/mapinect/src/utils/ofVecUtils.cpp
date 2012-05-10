#include "ofVecUtils.h"

#include "utils.h"

void findOfxVec3fBoundingBox(const std::vector<ofVec3f>& v, ofVec3f &vMin, ofVec3f &vMax) {
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

ofPolar cartesianToPolar(const ofPoint& c)
{
	ofPolar p;
	p.ro = sqrt(c.x * c.x + c.y * c.y);
	p.theta = 0;
	if (c.x != 0 || c.y != 0) {
		if (c.x == 0) {
			p.theta = c.y > 0 ? PI / 2 : - PI / 2;
		}
		else {
			p.theta = atan(c.y / c.x);
			if (c.x < 0) {
				p.theta += PI;
			}
		}
	}
	return p;
}

bool sortOnY(const ofVec3f& l, const ofVec3f& r) {
    return l.y < r.y;
}

bool sortOnX(const ofVec3f& l, const ofVec3f& r) {
    return l.x < r.x;
}

bool sortOnZ(const ofVec3f& l, const ofVec3f& r) {
    return l.z < r.z;
}
