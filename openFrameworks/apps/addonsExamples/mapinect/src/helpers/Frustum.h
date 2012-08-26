#ifndef FRUSTUM_H_
#define FRUSTUM_H_

#include "Plane3D.h"
#include "ofVec3f.h"

namespace mapinect
{
	class Frustum
	{
		public:
			static void RecalculateFrustum();
			static bool IsInFrustum(ofVec3f pto);
		private:
			Frustum();

			static Frustum*		instance;

			Plane3D				coneBottom;
			Plane3D				coneRightSide;
			Plane3D				coneTop;
			Plane3D				coneLeftSide;
	};
}

#endif	FRUSTUM_H_