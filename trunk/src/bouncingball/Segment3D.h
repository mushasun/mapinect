#ifndef Segment3D_H__
#define Segment3D_H__

#include "ofxVec3f.h"
class Segment3D {
	public:
		Segment3D(const ofxVec3f &origin, const ofxVec3f &destination,  const ofxVec3f &planeNormal, const ofxVec3f &normalDirection, bool inverseNormal = false, bool doubleNormal = false);
		virtual ~Segment3D() { };

		inline const ofxVec3f &getOrigin() { return pOrigin; }
		inline const ofxVec3f &getDestination() { return pDest; }
		inline const ofxVec3f &getDirection() { return direction; }
		inline const ofxVec3f &getNormal() { return normal; }
		inline const bool &isDoubleNormal() { return doubleNormal; }

		ofxVec3f closestPoint(ofxVec3f pto);
	private:
		ofxVec3f pOrigin;
		ofxVec3f pDest;
		ofxVec3f direction;
		ofxVec3f normal;
		bool doubleNormal;
	};

#endif	// MAPINECT_LINE2D_H__