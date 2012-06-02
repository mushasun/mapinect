#include "Polygon.h"

#include <algorithm>

#include "Globals.h"
#include "ofGraphicsUtils.h"
#include "ofVecUtils.h"
#include "SortPolar.h"
#include "utils.h"

namespace mapinect {

	Polygon::Polygon(const Polygon& polygon)
		: name(polygon.name), container(polygon.container), mathModel(polygon.mathModel)
	{
	}

	IPolygonPtr Polygon::clone() const
	{
		return IPolygonPtr(new Polygon(*this));
	}

	void Polygon::setVertex(int vertexNum, const ofVec3f& v)
	{
		mathModel.setVertex(vertexNum, v);
	}

	void Polygon::setVertexs(const vector<ofVec3f>& v)
	{
		vector<ofVec3f> vertexs(v);
		//sort(vertexs.begin(), vertexs.end(), SortPolar(vertexs));
		mathModel.setVertexs(vertexs);
	}

	void Polygon::draw() {
		//cout << calculateArea() << endl;
		glBegin(GL_POLYGON);
			for (vector<ofVec3f>::const_iterator v = mathModel.getVertexs().begin(); v != mathModel.getVertexs().end(); ++v)
			{
				glVertex3f(v->x, v->y, v->z);
			}
		glEnd();
		
		int i = 1;
		for (vector<ofVec3f>::const_iterator v = mathModel.getVertexs().begin(); v != mathModel.getVertexs().end(); ++v) {
			ofSetColor(0, 255 * i++ / 4.0f, 0);
			ofVec3f w = gKinect->getScreenCoordsFromWorldCoords(*v);
			ofCircle(w.x, w.y, 4);
		}
		
	}

}
