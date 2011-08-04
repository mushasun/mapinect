#ifndef MAPINECT_LINE2D_H__
#define MAPINECT_LINE2D_H__

#include "ofxVec2f.h"

namespace mapinect {

	typedef enum {
		kPositionedAtLeft,
		kPositionedInLine,
		kPositionedAtRight
	} PositionToLine;

	class Line2D {
	public:
		Line2D(const ofxVec2f &origin, const ofxVec2f &destination);
		virtual ~Line2D() { };

		inline const ofxVec2f &getOrigin() { return pOrigin; }

		float distance(const ofxVec2f &v);
		float calculateValue(const ofxVec2f &v);
		ofxVec2f projectTo(const ofxVec2f &v);
		PositionToLine positionTo(const ofxVec2f &f);

	private:
		ofxVec2f pOrigin;
		ofxVec2f pDirection;

		float pA, pB, pC;
	};
}

#endif	// MAPINECT_LINE2D_H__