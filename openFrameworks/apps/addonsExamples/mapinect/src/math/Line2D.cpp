#include "Line2D.h"
#include "utils.h"

namespace mapinect {

	Line2D::Line2D(const ofVec2f &origin, const ofVec2f &destination) {
		pOrigin = origin;
		pDirection = destination - origin;
		pDirection.normalize();
		pA = - pDirection.y;
		pB = pDirection.x;
		pC = origin.x * pDirection.y - origin.y * pDirection.x;
		pSqrtA2B2 = sqrt(pA * pA + pB * pB);
	}

	double Line2D::distance(const ofVec2f &v) {
		double num = abs(calculateValue(v));
		return num / pSqrtA2B2;
	}

	double Line2D::calculateValue(const ofVec2f &v) {
		return pA * v.x + pB * v.y + pC;
	}

	ofVec2f Line2D::projectTo(const ofVec2f &v) {
		ofVec2f vA = v - pOrigin;
		double u = vA.dot(pDirection);
		return ofVec2f(pOrigin.x + u * pDirection.x, pOrigin.y + u * pDirection.y);
	}

	PositionToLine Line2D::positionTo(const ofVec2f &v) {
		double value = calculateValue(v);
		if (abs(value) < DBL_EPSILON) {
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
