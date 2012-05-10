#ifndef MAPINECT_LINE2D_H__
#define MAPINECT_LINE2D_H__

#include "ofVec2f.h"
#include "ofVec3f.h"

namespace mapinect {

	typedef enum {
		kPositionedAtLeft,
		kPositionedInLine,
		kPositionedAtRight
	} PositionToLine;

	class Line2D {
	public:
		Line2D(const ofVec2f &origin, const ofVec2f &destination);
		virtual ~Line2D() { };

		inline const ofVec2f &getOrigin() { return pOrigin; }
		inline const ofVec3f getCoefficients() { return ofVec3f(pA,pB,pC); }

		double distance(const ofVec2f &v) const;
		double calculateValue(const ofVec2f &v) const;
		ofVec2f projectTo(const ofVec2f &v) const;
		PositionToLine positionTo(const ofVec2f &f) const;

	private:
		ofVec2f pOrigin;
		ofVec2f pDirection;

		double pA, pB, pC, pSqrtA2B2;
	};
}

#endif	// MAPINECT_LINE2D_H__