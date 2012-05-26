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
		Line2D(const ofVec2f& origin, const ofVec2f& destination);
		virtual ~Line2D() { };

		inline const ofVec2f& getOrigin() const			{ return origin; }
		inline const ofVec3f getCoefficients() const	{ return ofVec3f(a, b, c); }

		float distance(const ofVec2f& p) const;
		float projectedK(const ofVec2f& p) const;
		ofVec2f calculateValue(float k) const;
		ofVec2f projectTo(const ofVec2f& p) const;
		bool isInLine(const ofVec2f& p) const;
		bool isInSegment(const ofVec2f &p) const;
		bool isInSegment(float k) const;

		float evaluate(const ofVec2f& p) const;
		PositionToLine positionTo(const ofVec2f& p) const;

	private:
		ofVec2f origin;
		ofVec2f direction;
		ofVec2f	destination;

		float a, b, c, sqrtA2B2;
	};
}

#endif	// MAPINECT_LINE2D_H__