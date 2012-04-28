#include "Polygon.h"

#include <algorithm>
#include "utils.h"
#include "ofVecUtils.h"

namespace mapinect {

	const ofVec3f& Polygon::getVertex(int vertexNum) {
		return vertexs.at(vertexNum);
	}

	void Polygon::setVertex(int vertexNum, const ofVec3f& v) {
		vertexs[vertexNum] = v;
	}

	void Polygon::addVertex(const ofVec3f& v) {
		vertexs.push_back(v);
		if (vertexs.size() == 3) {
			normal = (vertexs.at(2) - vertexs.at(1)) * (vertexs.at(0) - vertexs.at(1));
			normal.normalize();
		}
	}

	static DiscardCoordinate	sortDiscardCoordinate;
	static ofVec3f				sortCenter;

	bool comparePolarCoords(const ofVec3f& v3A, const ofVec3f& v3B) {
		ofVec2f v2A = discardCoordinateOfxVec3f(v3A - sortCenter, sortDiscardCoordinate);
		ofVec2f v2B = discardCoordinateOfxVec3f(v3B - sortCenter, sortDiscardCoordinate);

		ofPolar pA = cartesianToPolar(v2A);
		ofPolar pB = cartesianToPolar(v2B);

		return pA.theta < pB.theta;
	}

	void Polygon::resetVertex() {
		vertexs.clear();
	}

	void Polygon::sortVertexs() {
		ofVec3f min, max;
		findOfxVec3fBoundingBox(vertexs, min, max);
		sortDiscardCoordinate = calculateDiscardCoordinate(min, max);
		sortCenter = (min + max) * 0.5;
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

	ofVec3f Polygon::project(const ofVec3f& p) {
		ofVec3f dif = p - getCenter();
		ofVec3f normalProj = dif.dot(normal) * normal;
		ofVec3f proj = p - normalProj;
		return proj;
	}

	void Polygon::draw() {
		//cout << calculateArea() << endl;
		glBegin(GL_POLYGON);
			for (int i = 0; i < vertexs.size(); i++) {
				glVertex3f(vertexs[i].x, vertexs[i].y, vertexs[i].z);
			}
		glEnd();
		
		int i = 1;
		for (vector<ofVec3f>::iterator iter = vertexs.begin(); iter != vertexs.end(); iter++) {
			ofSetColor(0,255 * i++ / 4.0f,0);
			ofVec3f w = gKinect->getScreenCoordsFromWorldCoords(*iter);
			ofCircle(w.x,w.y,4);
		}
		
	}


}

