#include "Polygon.h"

#include <algorithm>

#include "Globals.h"
#include "ofGraphicsUtils.h"
#include "ofVecUtils.h"
#include "SortPolar.h"
#include "utils.h"

namespace mapinect {

	void Polygon::setVertex(int vertexNum, const ofVec3f& v)
	{
		vertexs[vertexNum] = v;
	}

	void Polygon::setVertexs(const vector<ofVec3f>& v)
	{
		vertexs = v;
		mathPolygon = Polygon3D(vertexs);
	}

	void Polygon::sortVertexs()
	{
		sort(vertexs.begin(), vertexs.end(), SortPolar(vertexs));
		mathPolygon = Polygon3D(vertexs);
	}
	
	float Polygon::calculateArea()
	{
		return mathPolygon.calculateArea();
	}

	ofVec3f Polygon::project(const ofVec3f& p)
	{
		return mathPolygon.project(p);
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
			ofSetColor(0, 255 * i++ / 4.0f, 0);
			ofVec3f w = gKinect->getScreenCoordsFromWorldCoords(*iter);
			ofCircle(w.x,w.y,4);
		}
		
	}

	void Polygon::setName(const IPolygonName& n)
	{
		this->name = n;
	}


}

