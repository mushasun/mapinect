#include "Polyhedron.h"

namespace mapinect {

	void Polyhedron::addPolygon(Polygon* polygon) {
		polygons.push_back(polygon);
	}

	Polygon* Polyhedron::getPolygonAt(int index) {
		return polygons.at(index);
	}

	const ofxVec3f& Polyhedron::getVertex(int index) {
		return ofxVec3f();
	}

	void Polyhedron::draw() {
		for (vector<Polygon*>::iterator iter = polygons.begin(); iter != polygons.end(); iter++) {
			(*iter)->drawObject();
		}
	}

}
