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
			ro = sqrt(cartesian.x * cartesian.x + cartesian.y * cartesian.y);
			theta = 0;
			if (cartesian.x != 0 || cartesian.y != 0) {
				if (cartesian.x == 0) {
					theta = cartesian.y > 0 ? PI / 2 : - PI / 2;
				}
				else {
					theta = atan(cartesian.y / cartesian.x);
					if (cartesian.x < 0) {
						theta += PI;
					}
				}
			}
		}
		
		float	ro;
		float	theta;

	};
}

#endif	// MAPINECT_OFPOLAR_H__
