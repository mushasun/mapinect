#include "Sphere.h"

#define SLICES		32
#define STACKS		8

namespace mapinect {

	Sphere::Sphere(float r) : radius(r) {
		gluQuad = gluNewQuadric();
	}

	void Sphere::draw() {
		gluSphere(gluQuad, radius, SLICES, STACKS);
	}

}
