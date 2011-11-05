#include "Polygon.h"

#include <algorithm>
#include "utils.h"
#include "ofxVecUtils.h"

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

	DiscardCoordinate	sortDiscardCoordinate;
	ofxVec3f			sortCenter;

	bool comparePolarCoords(const ofxVec3f& v3A, const ofxVec3f& v3B) {
		ofxVec2f v2A = discardCoordinateOfxVec3f(v3A - sortCenter, sortDiscardCoordinate);
		ofxVec2f v2B = discardCoordinateOfxVec3f(v3B - sortCenter, sortDiscardCoordinate);

		double phiA = 0;
		if (v2A.x != 0 || v2A.y != 0) {
			double roA = sqrt(v2A.x * v2A.x + v2A.y * v2A.y);
			double arcsinA = asin(v2A.y / roA);
			if (v2A.x >= 0) {
				phiA = arcsinA;
			}
			else {
				phiA = PI - arcsinA;
			}
		}

		double phiB = 0;
		if (v2B.x != 0 || v2B.y != 0) {
			double roB = sqrt(v2B.x * v2B.x + v2B.y * v2B.y);
			double arcsinB = asin(v2B.y / roB);
			if (v2B.x >= 0) {
				phiB = arcsinB;
			}
			else {
				phiB = PI - arcsinB;
			}
		}

		return phiA < phiB;
	}

	void Polygon::resetVertex() {
		vertexs.clear();
	}

	void Polygon::sortVertexs() {
		ofxVec3f min, max;
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

