#include "Canvas.h"

#include <assert.h>
#include "ofGraphicsUtils.h"

namespace drawing {
	
	Canvas::Canvas(const IPolygonPtr& polygon, const ofColor& backColor, const ofColor& foreColor)
		: polygon(polygon), backColor(backColor), foreColor(foreColor), needsToRedraw(true)
	{
		int vertexCount = polygon->getMathModel().getVertexs().size();
		assert(vertexCount >= 3);
		int origin = polygon->getBestOriginVertexIndex();

		const double pxScale = 300;
		dimensions.x = floor(polygon->getMathModel().getEdges()[origin].segmentLength() * pxScale);
		dimensions.y = floor(polygon->getMathModel().getEdges()[(origin + vertexCount - 1) % vertexCount].segmentLength() * pxScale);

		texture.setup((int)dimensions.x, (int)dimensions.y);
		texture.background(backColor);

		update(polygon);
	}

	Canvas::~Canvas()
	{
	}

	void Canvas::setBackColor(const ofColor& color)
	{
		backColor = color;
		redraw();
	}

	void Canvas::setForeColor(const ofColor& color)
	{
		foreColor = color;
	}

	void Canvas::update(const IPolygonPtr& polygon)
	{
		this->polygon = polygon;

		int vertexCount = polygon->getMathModel().getVertexs().size();
		assert(vertexCount >= 3);
		int origin = polygon->getBestOriginVertexIndex();

		texCoords.clear();
		texCoords.push_back(ofVec2f(0, 0));
		texCoords.push_back(ofVec2f(dimensions.x, 0));
		texCoords.push_back(ofVec2f(dimensions.x, dimensions.y));
		texCoords.push_back(ofVec2f(0, dimensions.y));

		texMapper = TextureMapper2D(polygon->getMathModel(), texCoords, origin);
	}

	void Canvas::redraw()
	{
		needsToRedraw = true;
	}

	void Canvas::redrawIfNecessary()
	{
		if (needsToRedraw)
		{
			texture.background(backColor);

			for (map<int, IDrawer*>::iterator d = drawers.begin(); d != drawers.end(); ++d)
			{
				d->second->draw(texture);
			}

			texture.update();
			needsToRedraw = false;
		}
	}

	void Canvas::draw()
	{
		ofImage* cairoTexture = texture.getTextureRef();
		cairoTexture->bind();
		ofDrawQuadTextured(polygon->getMathModel().getVertexs(), texCoords);
		cairoTexture->unbind();

		for (map<int, DataTouch>::const_iterator t = touchPoints.begin(); t != touchPoints.end(); ++t)
		{
			ofSetColor(kRGBBlue);
			ofVec3f tp(t->second.getTouchPoint());
			ofCircle(tp.x, tp.y, tp.z, 0.003);
		}
	}

	void Canvas::touchEvent(const DataTouch& touchPoint)
	{
		assert(touchPoint.getPolygon()->getId() == polygon->getId());
		int id = touchPoint.getId();
		map<int, IDrawer*>::iterator d = drawers.find(id);
		switch (touchPoint.getType())
		{
		case kTouchTypeStarted:
			touchPoints[id] = touchPoint;
			drawers[id] = IDrawer::SCreate(mapToTexture(touchPoint.getTouchPoint()), foreColor);
			setForeColor(ofRandomColor());
			break;
		case kTouchTypeHolding:
			touchPoints[id] = touchPoint;
			if (d != drawers.end())
			{
				d->second->update(mapToTexture(touchPoint.getTouchPoint()));
				d->second->draw(texture);
				redraw();
			}
			break;
		case kTouchTypeReleased:
			touchPoints.erase(id);
			break;
		}
	}
}
