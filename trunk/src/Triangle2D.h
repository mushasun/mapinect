#ifndef MAPINECT_TRIANGLE2D_H__
#define MAPINECT_TRIANGLE2D_H__

#include "ofxVec2f.h"
#include "Line2D.h"

namespace mapinect {
	class Triangle2D {
	public:
		Triangle2D(const ofxVec2f &vA, const ofxVec2f &vB, const ofxVec2f &vC);
		virtual ~Triangle2D() { };

		float distance(const ofxVec2f& v);

	private:
		Line2D			pAB, pBC, pCA;
		PositionToLine	pSign;
	};
}

#endif	// MAPINECT_TRIANGLE2D_H__
