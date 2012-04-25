#include "Segment3D.h"

namespace bouncing {
	Segment3D::Segment3D(const ofVec3f &origin, const ofVec3f &destination,  const ofVec3f &planeNormal, const ofVec3f &normalDirection, bool inverseNormal, bool isDoubleNormal)
	{
		pOrigin = origin;
		pDest = destination;
		direction = destination - origin;
		direction = direction.normalize();
		normal = planeNormal.getCrossed(direction).normalize();
		doubleNormal = isDoubleNormal;

		float dist = ((normal+pOrigin) - normalDirection).length();
		if(inverseNormal)
		{
			if(((-normal+pOrigin) - normalDirection).length() > dist)
				normal *= -1;
		}
		else
			if(((-normal+pOrigin) - normalDirection).length() < dist)
				normal *= -1;
	}

	ofVec3f Segment3D::closestPoint(ofVec3f pto)
	{
		ofVec3f v = pDest - pOrigin; 
		ofVec3f w = pto - pOrigin;
		float t = w.dot(v) / v.dot(v);
		t = CLAMP(t,0,1);
		return pOrigin + (v * t);
	}

}
