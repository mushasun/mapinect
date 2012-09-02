#include "Polygon.h"

#include <algorithm>

#include "Globals.h"
#include "ofGraphicsUtils.h"
#include "ofVecUtils.h"
#include "SortPolar.h"
#include "utils.h"
#include "pointUtils.h"
#include "transformationUtils.h"

namespace mapinect {

	Polygon::Polygon(const Polygon& polygon)
		: name(polygon.name), container(polygon.container), mathModel(polygon.mathModel)
	{
		this->setId(polygon.getId());
	}

	IPolygonPtr Polygon::clone() const
	{
		IPolygonPtr p (new Polygon(*this));
		return p;
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
		mathModel.setVertexs(vertexs);
	}

	void Polygon::setPlane(const Plane3D& p) 
	{
		mathModel.setPlane(p);
	}

	void Polygon::setVertexsOrdered(const vector<ofVec3f>& v)
	{
		vector<ofVec3f> vertexs(v);
		vector<ofVec3f> oldVertexs = mathModel.getVertexs();
		vector<ofVec3f> orderedVertexs;

		for(int i = 0; i < oldVertexs.size(); i++)
		{
			vector<ofVec3f>::const_iterator idx = findCloser(oldVertexs.at(i), vertexs);
			orderedVertexs.push_back(*idx);
			vertexs.erase(idx);
		}
		mathModel.setVertexs(orderedVertexs);
	}

	void Polygon::draw() {
		//cout << calculateArea() << endl;
		if(this->getId() == TABLE_ID)
		{
			vector<char> tableVertexs;
			tableVertexs.push_back('A');
			tableVertexs.push_back('B');
			tableVertexs.push_back('C');
			tableVertexs.push_back('D');
			ofVec3f prev;
			for (int i = 0; i < mathModel.getVertexs().size(); i++) {
				ofVec3f w = getScreenCoords(mathModel.getVertexs().at(i));
				ofSetColor(kRGBBlue);
				ofDrawBitmapString(ofToString(tableVertexs.at(i)), w.x + 10, w.y + 10, 0);
				prev.z = 0;
				w.z = 0;
				ofSetColor(kRGBGreen);
				ofCircle(w.x, w.y, 4, 4);
				if (i != 0) {
					ofLine(prev,w);
				}
				prev = w;
			}

			if (mathModel.getVertexs().size() > 0) {
				ofVec3f first = getScreenCoords(mathModel.getVertexs().at(0));
				prev.z = 0;
				first.z = 0;
				ofLine(prev, first);
			}
		}
	}
}
