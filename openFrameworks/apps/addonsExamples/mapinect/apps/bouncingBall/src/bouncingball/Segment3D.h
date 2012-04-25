#ifndef Segment3D_H__
#define Segment3D_H__

#include "ofVec3f.h"

namespace bouncing {
	class Segment3D {
		public:
			Segment3D(const ofVec3f &origin, const ofVec3f &destination,  const ofVec3f &planeNormal, const ofVec3f &normalDirection, bool inverseNormal = false, bool doubleNormal = false);
			virtual ~Segment3D() { };

			inline const ofVec3f &getOrigin() { return pOrigin; }
			inline const ofVec3f &getDestination() { return pDest; }
			inline const ofVec3f &getDirection() { return direction; }
			inline const ofVec3f &getNormal() { return normal; }
			inline const bool &isDoubleNormal() { return doubleNormal; }

			ofVec3f closestPoint(ofVec3f pto);
		private:
			ofVec3f pOrigin;
			ofVec3f pDest;
			ofVec3f direction;
			ofVec3f normal;
			bool doubleNormal;
	};
}

#endif	// MAPINECT_LINE2D_H__