#ifndef MAPINECT_LINE3D_H__
#define MAPINECT_LINE3D_H__

#include "ofVec3f.h"
#include "ofVecUtils.h"
#include "Line3D.h"

namespace mapinect {

	class Line3D {
	public:
		Line3D(const ofVec3f &origin, const ofVec3f &destination);
		virtual ~Line3D() { };

		inline bool				isValid() const			{ return valid; }
		inline const ofVec3f&	getOrigin() const		{ return origin; }
		inline const ofVec3f&	getDirection() const	{ return direction; }
		inline const ofVec3f&	getDestination() const	{ return destination; }

		float distance(const ofVec3f &p) const;
		float projectedK(const ofVec3f& p) const;
		ofVec3f calculateValue(float k) const;
		ofVec3f projectTo(const ofVec3f &p) const;
		bool isInLine(const ofVec3f& p) const;
		bool isInSegment(const ofVec3f &p) const;
		bool isInSegment(float k) const;

	private:
		bool		valid;
		ofVec3f		origin;
		ofVec3f		destination;
		ofVec3f		direction;
	};

}

#endif	// MAPINECT_LINE3D_H__