#include "Line2D.h"
#include "utils.h"

namespace mapinect {

	Line2D::Line2D(const ofxVec2f &origin, const ofxVec2f &destination) {
		pOrigin = origin;
		pDirection = destination - origin;
		pDirection.normalize();
		pA = - pDirection.y;
		pB = pDirection.x;
		pC = origin.x * pDirection.y - origin.y * pDirection.x;
		pSqrtA2B2 = sqrt(pA * pA + pB * pB);
	}

	double Line2D::distance(const ofxVec2f &v) {
		double num = dabsd(calculateValue(v));
		return num / pSqrtA2B2;
	}

	double Line2D::calculateValue(const ofxVec2f &v) {
		return pA * v.x + pB * v.y + pC;
	}

	ofxVec2f Line2D::projectTo(const ofxVec2f &v) {
		ofxVec2f vA = v - pOrigin;
		double u = vA.dot(pDirection);
		return ofxVec2f(pOrigin.x + u * pDirection.x, pOrigin.y + u * pDirection.y);
	}

	PositionToLine Line2D::positionTo(const ofxVec2f &v) {
		double value = calculateValue(v);
		if (dabsd(value) < DBL_EPSILON) {
			return kPositionedInLine;
		}
		else if (value < 0) {
			return kPositionedAtLeft;
		}
		else {
			return kPositionedAtRight;
		}
	}

}
