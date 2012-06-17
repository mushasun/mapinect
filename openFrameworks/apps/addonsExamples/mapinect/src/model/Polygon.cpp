#include "Polygon.h"

#include <algorithm>

#include "Globals.h"
#include "ofGraphicsUtils.h"
#include "ofVecUtils.h"
#include "SortPolar.h"
#include "utils.h"
#include "pointUtils.h"

namespace mapinect {

	Polygon::Polygon(const Polygon& polygon)
		: name(polygon.name), container(polygon.container), mathModel(polygon.mathModel)
	{
	}

	IPolygonPtr Polygon::clone() const
	{
		return IPolygonPtr(new Polygon(*this));
	}

	int Polygon::getBestOriginVertexIndex() const
	{
		switch (name)
		{
		case kPolygonNameTop:
			return 1;
		default:
			return 3;
		}
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
		if(this->getId() == TABLE_ID)
		{
			for (vector<ofVec3f>::const_iterator v = mathModel.getVertexs().begin(); v != mathModel.getVertexs().end(); ++v) {
				ofSetColor(kRGBGreen);
				ofVec3f w = getScreenCoords(*v);
				ofCircle(w.x, w.y, 4, 4);
			}
		}
	}
}
