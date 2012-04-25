#include "Cylinder.h"

#define SLICES		32
#define STACKS		8

namespace mapinect {
	
	Cylinder::Cylinder(float h, float b) : height(h), baseRadius(b), topRadius(b) {
		gluQuad = gluNewQuadric();
	}

	Cylinder::Cylinder(float h, float b, float t) : height(h), baseRadius(b), topRadius(t) {
		gluQuad = gluNewQuadric();
	}

	void Cylinder::draw() {
		gluCylinder(gluQuad, baseRadius, topRadius, height, SLICES, STACKS);
	}

}
