#ifndef MAPINECT_TRIANGLE2D_H__
#define MAPINECT_TRIANGLE2D_H__

#include "ofVec2f.h"
#include "Line2D.h"

namespace mapinect {
	class Triangle2D {
	public:
		Triangle2D(const ofVec2f &vA, const ofVec2f &vB, const ofVec2f &vC);
		virtual ~Triangle2D() { };

		double distance(const ofVec2f& v);

	private:
		Line2D			pAB, pBC, pCA;
		PositionToLine	pSign;
	};
}

#endif	// MAPINECT_TRIANGLE2D_H__
