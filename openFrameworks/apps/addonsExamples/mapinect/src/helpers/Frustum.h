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
			static bool IsInFrustum(const ofVec3f& pto);
			static bool IsInFrustum(const vector<ofVec3f>& vertices);
			static void drawFrustum();	
			static void debugDrawFrustum();
		private:
			Frustum();

			static Frustum*		instance;

			Plane3D				coneBottom;
			Plane3D				coneRightSide;
			Plane3D				coneTop;
			Plane3D				coneLeftSide;

			// Los vértices del frustum, en coordenadas de pantalla
			ofVec3f sftl, sftr, sfbl, sfbr, sntl, sntr, snbl, snbr;	

			// Los vértices del frustum, en coordenadas de mundo
			ofVec3f ftl, ftr, fbl, fbr, ntl, ntr, nbl, nbr;	
	};
}

#endif	FRUSTUM_H_