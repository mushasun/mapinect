#include "Line2D.h"

namespace mapinect {

	Line2D::Line2D(const ofxVec2f &origin, const ofxVec2f &destination) {
		pOrigin = origin;
		pDirection = destination - origin;
		pDirection.normalize();
		pA = pDirection.y - pOrigin.y;
		pB = pOrigin.x - pDirection.y;
		pC = pOrigin.y * pB - pOrigin.x * pA;
	}

	float Line2D::distance(const ofxVec2f &v) {
		float denom = sqrt(pA * pA + pB * pB);
		float num = fabsf(calculateValue(v));
		return num / denom;
	}

	float Line2D::calculateValue(const ofxVec2f &v) {
		return pA * v.x + pB * v.y + pC;
	}

	ofxVec2f Line2D::projectTo(const ofxVec2f &v) {
		float u = v.x - pOrigin.x + v.y - pOrigin.y;
		return ofxVec2f(pOrigin.x + u * pDirection.x, pOrigin.y + u * pDirection.y);
	}

	PositionToLine Line2D::positionTo(const ofxVec2f &v) {
		float value = calculateValue(v);
		if (value == 0) {
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
