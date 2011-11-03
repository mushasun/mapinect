#ifndef MAPINECT_CYLINDER_H__
#define MAPINECT_CYLINDER_H__

#include "ModelObject.h"
#include "glu.h"

namespace mapinect {

	class Cylinder : public ModelObject {
		public:
			Cylinder(float height, float baseRadius);
			Cylinder(float height, float baseRadius, float topRadius);
			virtual ~Cylinder() { }

			virtual void draw();
	
		private:
			float			height;
			float			baseRadius, topRadius;
			GLUquadric*		gluQuad;

	};
}

#endif	// MAPINECT_CYLINDER_H__