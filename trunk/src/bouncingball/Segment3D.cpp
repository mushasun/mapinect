#include "Segment3D.h"

Segment3D::Segment3D(const ofxVec3f &origin, const ofxVec3f &destination,  const ofxVec3f &planeNormal, const ofxVec3f &normalDirection, bool inverseNormal, bool isDoubleNormal)
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

ofxVec3f Segment3D::closestPoint(ofxVec3f pto)
{
	ofxVec3f v = pDest - pOrigin; 
	ofxVec3f w = pto - pOrigin;
	float t = w.dot(v) / v.dot(v);
	t = CLAMP(t,0,1);
	return pOrigin + (v * t);
}

	