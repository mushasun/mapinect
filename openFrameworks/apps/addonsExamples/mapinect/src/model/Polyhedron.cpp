#include "Polyhedron.h"

namespace mapinect {

	void Polyhedron::addPolygon(const PolygonPtr& polygon) {
		polygons.push_back(polygon);
	}

	const PolygonPtr& Polyhedron::getPolygonAt(int index) {
		return polygons.at(index);
	}

	const ofVec3f& Polyhedron::getVertex(int index) {
		return ofVec3f();
	}

	void Polyhedron::draw() {
		for (vector<PolygonPtr>::iterator iter = polygons.begin(); iter != polygons.end(); iter++) {
			(*iter)->drawObject();
		}
	}

}
