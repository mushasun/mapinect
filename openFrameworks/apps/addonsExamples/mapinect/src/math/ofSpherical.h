#ifndef MAPINECT_OFSPHERICAL_H__
#define MAPINECT_OFSPHERICAL_H__

#include "ofVec3f.h"

namespace mapinect
{
	struct ofSpherical
	{
	public:
		ofSpherical(float ro, float theta, float phi)
			: ro(ro), theta(theta), phi(phi) { }

		ofSpherical(const ofVec3f& cartesian)
		{
			// http://www.thecubiclewarrior.com/post/5954842175/spherical-to-cartesian
			ro = cartesian.length();
			theta = atan2(cartesian.x, cartesian.z);
			phi = acos(cartesian.y / ro);
		}
		
		ofVec3f toCartesian()
		{
			// http://www.thecubiclewarrior.com/post/5954842175/spherical-to-cartesian
			return ofVec3f(ro * sin(phi) * sin(theta), ro * cos(phi), ro * sin(phi) * cos(theta));
		}

		float	ro;
		float	theta;
		float	phi;

	};
}

#endif	// MAPINECT_OFSPHERICAL_H__
