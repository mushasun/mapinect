#include "Canvas.h"

#include <assert.h>
#include "ofGraphicsUtils.h"

namespace drawing {
	
	Canvas::Canvas(ITxManager* txManager, const IPolygonPtr& polygon, int backColor, int foreColor)
		: polygon(polygon), backColor(backColor), foreColor(foreColor)
	{
		int vertexCount = polygon->getMathModel().getVertexs().size();
		assert(vertexCount >= 3);
		int origin = polygon->getBestOriginVertexIndex();

		dimensions.x = floor(polygon->getMathModel().getEdges()[origin].segmentLength());
		dimensions.y = floor(polygon->getMathModel().getEdges()[(origin + vertexCount - 1) % vertexCount].segmentLength());

		texCoords.push_back(ofVec2f(0, 0));
		texCoords.push_back(ofVec2f(dimensions.x, 0));
		texCoords.push_back(ofVec2f(dimensions.x, dimensions.y));
		texCoords.push_back(ofVec2f(0, dimensions.y));

		texMapper = TextureMapper2D(polygon->getMathModel(), texCoords, origin);
		
		texture.setup((int)dimensions.x, (int)dimensions.y);
		texture.background(backColor);

		
	}

	Canvas::~Canvas()
	{
	}

	void Canvas::setBackColor(int color)
	{
		backColor = color;
	}

	void Canvas::setForeColor(int color)
	{
		foreColor = color;
	}

	void Canvas::draw()
	{

		ofDrawQuadTextured(polygon->getMathModel().getVertexs(), texCoords);
	}

	void Canvas::touchEvent(const DataTouch& touchPoint)
	{
		assert(touchPoint.getPolygon()->getId() == polygon->getId());

	}
}
