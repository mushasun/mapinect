#include "Triangle2D.h"

namespace mapinect {
	Triangle2D::Triangle2D(const ofVec2f &vA, const ofVec2f &vB, const ofVec2f &vC)
		: pAB(vA, vB), pBC(vB, vC), pCA(vC, vA) {
		pSign = pAB.positionTo(vC);
		pSign = pBC.positionTo(vA);
		pSign = pCA.positionTo(vB);
	}

	double Triangle2D::distance(const ofVec2f &v) {
		int sameHalfPlanes = 0;
		Line2D *line;
		ofVec2f end;
		if (pAB.positionTo(v) == pSign) {
			sameHalfPlanes++;
		}
		else {
			line = &pAB;
			end = pBC.getOrigin();
		}
		if (pBC.positionTo(v) == pSign) {
			sameHalfPlanes++;
		}
		else {
			line = &pBC;
			end = pCA.getOrigin();
		}
		if (pCA.positionTo(v) == pSign) {
			sameHalfPlanes++;
		}
		else {
			line = &pCA;
			end = pAB.getOrigin();
		}

		if (sameHalfPlanes == 3) {
			// point is inside the triangle
			return 0;
		}
		else if (sameHalfPlanes == 2) {
			// point could have a projection on a side
			ofVec2f vProjected = line->projectTo(v);
			bool vProjectedInSegment =
				((line->getOrigin().x <= vProjected.x && vProjected.x <= end.x)
				|| (line->getOrigin().x >= vProjected.x && vProjected.x >= end.x))
				&&
				((line->getOrigin().y <= vProjected.y && vProjected.y <= end.y)
				|| (line->getOrigin().y >= vProjected.y && vProjected.y >= end.y));

			if (vProjectedInSegment) {
				return line->distance(v);
			}
		}
		return MIN(MIN(pAB.getOrigin().distance(v), pBC.getOrigin().distance(v)), pCA.getOrigin().distance(v));
	}

}
