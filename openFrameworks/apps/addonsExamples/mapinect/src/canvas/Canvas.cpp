#include "Canvas.h"

#include <assert.h>
#include "ofGraphicsUtils.h"

namespace mapinect
{
	const int kMappingOriginVertex = 1;

	Canvas::Canvas(int polygonId, const Polygon3D& polygon, int width, int height, const ofColor& backColor, const ofColor& foreColor, float lineWidth)
		: polygonId(polygonId), polygon(polygon), width(width), height(height), backColor(backColor), foreColor(foreColor), needsToRedraw(true), lineWidth(lineWidth)
	{
		int vertexCount = polygon.getVertexs().size();
		assert(vertexCount >= 3);

		texture.setup(width, height);
		texture.background(backColor);
		texture.setLineWidth(lineWidth);
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

	void Canvas::setLineWidth(float width)
	{
		lineWidth = width;
		texture.setLineWidth(lineWidth);
	}

	void Canvas::update(const Polygon3D& polygon)
	{
		this->polygon = polygon;

		int vertexCount = polygon.getVertexs().size();
		assert(vertexCount >= 3);

		texCoords.clear();
		texCoords.push_back(ofVec2f(0, 0));
		texCoords.push_back(ofVec2f(width, 0));
		texCoords.push_back(ofVec2f(width, height));
		texCoords.push_back(ofVec2f(0, height));

		texMapper = TextureMapper2D(polygon, texCoords, kMappingOriginVertex);
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
		redrawIfNecessary();
		ofImage* cairoTexture = texture.getTextureRef();
		cairoTexture->bind();
		ofDrawQuadTextured(polygon.getVertexs(), texCoords);
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
		assert(touchPoint.getPolygon()->getId() == polygonId);
		if (texMapper.willMap(touchPoint.getTouchPoint()))
		{
			ofVec3f mapped(texMapper.map(touchPoint.getTouchPoint()));
			int id = touchPoint.getId();
			map<int, IDrawer*>::iterator d = drawers.find(id);
			switch (touchPoint.getType())
			{
			case kTouchTypeStarted:
				touchPoints[id] = touchPoint;
				drawers[id] = IDrawer::SCreate(mapped, foreColor);
				//setForeColor(ofRandomColor());
				break;
			case kTouchTypeHolding:
				touchPoints[id] = touchPoint;
				if (d != drawers.end())
				{
					d->second->update(mapped);
					redraw();
				}
				break;
			case kTouchTypeReleased:
				touchPoints.erase(id);
				break;
			}
		}
	}
}
