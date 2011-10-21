#include "Polygon.h"
#include <algorithm>

namespace mapinect {

	const ofxVec3f& Polygon::getVertex(int vertexNum) {
		return vertexs.at(vertexNum);
	}

	void Polygon::addVertex(ofxVec3f v) {
		vertexs.push_back(v);
	}

	bool comparePolarCoords(const ofxVec3f& v1, const ofxVec3f& v2) {
		double phi1 = 0;
		if (v1.x != 0 || v1.y != 0) {
			double ro1 = sqrt(v1.x * v1.x + v1.y * v1.y);
			double arcsin1 = asin(v1.y / ro1);
			if (v1.x >= 0) {
				phi1 = arcsin1;
			}
			else {
				phi1 = PI - arcsin1;
			}
		}

		double phi2 = 0;
		if (v2.x != 0 || v2.y != 0) {
			double ro2 = sqrt(v2.x * v2.x + v2.y * v2.y);
			double arcsin2 = asin(v2.y / ro2);
			if (v2.x >= 0) {
				phi2 = arcsin2;
			}
			else {
				phi2 = PI - arcsin2;
			}
		}

		return phi1 < phi2;
	}

	void Polygon::sortVertexs() {
		sort(vertexs.begin(), vertexs.end(), comparePolarCoords);
	}

	void Polygon::draw() {
		glBegin(GL_POLYGON);
			for (vector<ofxVec3f>::iterator iter = vertexs.begin(); iter != vertexs.end(); iter++) {
				glVertex3f(iter->x, iter->y, iter->z);
			}
		glEnd();
	}

}

