#include "Box.h"

namespace mapinect {
	Box::Box(const ofxVec3f& dimensions) {

	}

	const ofxVec3f& Box::getVertex(BoxVertex v) {
		return Polyhedron::getVertex((int)v);
	}
}
