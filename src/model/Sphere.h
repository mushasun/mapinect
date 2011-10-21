#ifndef MAPINECT_SPHERE_H__
#define MAPINECT_SPHERE_H__

#include "ModelObject.h"
#include "glu.h"

namespace mapinect {
	class Sphere : public ModelObject {
		public:
			Sphere(float r);
			virtual ~Sphere() { }

			virtual void draw();
	
		private:
			float			radius;
			GLUquadric*		gluQuad;

	};
}

#endif	// MAPINECT_SPHERE_H__