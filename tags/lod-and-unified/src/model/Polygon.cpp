#include "Polygon.h"
#include <algorithm>
#include "utils.h"

namespace mapinect {

	const ofxVec3f& Polygon::getVertex(int vertexNum) {
		return vertexs.at(vertexNum);
	}

	void Polygon::setVertex(int vertexNum, const ofxVec3f& v) {
		vertexs[vertexNum] = v;
	}

	void Polygon::addVertex(const ofxVec3f& v) {
		vertexs.push_back(v);
		if (vertexs.size() == 3) {
			normal = (vertexs.at(2) - vertexs.at(1)) * (vertexs.at(0) - vertexs.at(1));
			normal.normalize();
		}
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

	void Polygon::resetVertex() {
		vertexs.clear();
	}

	void Polygon::sortVertexs() {
		sort(vertexs.begin(), vertexs.end(), comparePolarCoords);
	}
	
	float Polygon::calculateArea() {
		// Code taken from http://softsurfer.com/Archive/algorithm_0101/algorithm_0101.htm#area3D_Polygon()
		float area = 0;
		float ax, ay, az;		// abs value of normal and its coords
		int   coord;			// coord to ignore: 1=x, 2=y, 3=z
		int   i, j, k;			// loop indices

		// select largest abs coordinate to ignore for projection
		ax = abs(normal.x);     // abs x-coord
		ay = abs(normal.y);     // abs y-coord
		az = abs(normal.z);     // abs z-coord

		coord = 3;                     // ignore z-coord
		if (ax > ay) {
			if (ax > az) coord = 1;    // ignore x-coord
		}
		else if (ay > az) coord = 2;   // ignore y-coord

		
		vertexs.push_back(vertexs.at(0));
		vertexs.push_back(vertexs.at(1));
		
		// compute area of the 2D projection
		for (i = 1, j = 2, k = 0; i <= vertexs.size() - 2; i++, j++, k++) {
			switch (coord) {
			case 1:
				area += (vertexs[i].y * (vertexs[j].z - vertexs[k].z));
				continue;
			case 2:
				area += (vertexs[i].x * (vertexs[j].z - vertexs[k].z));
				continue;
			case 3:
				area += (vertexs[i].x * (vertexs[j].y - vertexs[k].y));
				continue;
			}
		}

		vertexs.erase(vertexs.begin() + (vertexs.size() - 1));
		vertexs.erase(vertexs.begin() + (vertexs.size() - 1));
		
		// scale to get area before projection
		switch (coord) {
		case 1:
			area *= (1.0 / (2*ax));
			break;
		case 2:
			area *= (1.0 / (2*ay));
			break;
		case 3:
			area *= (1.0 / (2*az));
		}
		return abs(area);
	}

	void Polygon::draw() {
		//cout << calculateArea() << endl;
		/*glBegin(GL_POLYGON);
			for (vector<ofxVec3f>::iterator iter = vertexs.begin(); iter != vertexs.end(); iter++) {
				ofxVec3f w = gKinect->getScreenCoordsFromWorldCoords(*iter);
				glVertex3f(w.x, w.y, 4);
			}
		glEnd();*/

		int i = 1;
		for (vector<ofxVec3f>::iterator iter = vertexs.begin(); iter != vertexs.end(); iter++) {
			ofSetColor(0,255 * i++ / 4.0f,0);
			ofxVec3f w = gKinect->getScreenCoordsFromWorldCoords(*iter);
			ofCircle(w.x,w.y,4);
		}
	}


}

