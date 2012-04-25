#include "Box.h"

namespace mapinect {
	Box::Box(const ofVec3f& dimensions) {

	}

	const ofVec3f& Box::getVertex(BoxVertex v) {
		return Polyhedron::getVertex((int)v);
	}
}
