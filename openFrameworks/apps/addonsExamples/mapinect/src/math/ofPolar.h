#ifndef MAPINECT_OFPOLAR_H__
#define MAPINECT_OFPOLAR_H__

#include "ofVec2f.h"

namespace mapinect
{
	struct ofPolar
	{
	public:
		ofPolar(float ro, float theta)
			: ro(ro), theta(theta) { }

		ofPolar(const ofVec2f& cartesian)
		{
			ro = cartesian.length();
			theta = atan2(cartesian.x, cartesian.y);
		}

		ofVec2f toCartesian()
		{
			return ofVec2f(ro * cos(theta), ro * sin(theta));
		}
		
		float	ro;
		float	theta;

	};
}

#endif	// MAPINECT_OFPOLAR_H__
