#include "ofVecUtils.h"

#include "utils.h"

ofVec3f BAD_OFVEC3F(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);

void computeBoundingBox(const std::vector<ofVec3f>& v, ofVec3f& min, ofVec3f& max) {
	min = ofVec3f(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
	max = ofVec3f(-MAX_FLOAT, -MAX_FLOAT, -MAX_FLOAT);

	for (int k = 0; k < v.size(); k++) {
		ofVec3f p = v.at(k);
		min.x = minf(p.x, min.x);
		min.y = minf(p.y, min.y);
		min.z = minf(p.z, min.z);
		max.x = maxf(p.x, max.x);
		max.y = maxf(p.y, max.y);
		max.z = maxf(p.z, max.z);
	}

}
