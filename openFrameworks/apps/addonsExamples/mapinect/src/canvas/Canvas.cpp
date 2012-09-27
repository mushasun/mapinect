#include "Canvas.h"

#include <assert.h>
#include "ofGraphicsUtils.h"

namespace mapinect
{
	const int kMappingOriginVertex = 1;

	Canvas::Canvas(int polygonId, const Polygon3D& polygon, int width, int height,
		const ofColor& backColor, const ofColor& foreColor, float lineWidth)
		: polygonId(polygonId), polygon(polygon), width(width), height(height),
		backColor(backColor), foreColor(foreColor), needsToRedraw(true), lineWidth(lineWidth)
	{
		int vertexCount = polygon.getVertexs().size();
		assert(vertexCount >= 3);

		texture.setup(width, height);
		texture.background(backColor);
		texture.setLineWidth(lineWidth);
		update(polygon);
		lastPoint = ofVec3f(numeric_limits<float>::min(),numeric_limits<float>::min(),numeric_limits<float>::min());
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

		texMapper = TextureMapper2D(polygon, ofTexCoordsFor(width, height), kMappingOriginVertex);
	}

	void Canvas::redraw()
	{
		needsToRedraw = true;
	}

	void Canvas::redrawIfNecessary()
	{
		if (needsToRedraw)
		{
			for (map<int, Trace*>::iterator t = traces.begin(); t != traces.end(); ++t)
			{
				t->second->draw(texture);
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
		ofDrawQuadTextured(polygon.getVertexs(), ofTexCoordsFor());
		cairoTexture->unbind();

		for (map<int, DataTouch>::const_iterator t = touchPoints.begin(); t != touchPoints.end(); ++t)
		{
			ofSetColor(kRGBBlue);
			ofVec3f tp(t->second.getTouchPoint());
			ofCircle(tp.x, tp.y, tp.z, 0.003);
		}
	}

	void Canvas::drawTexture(ofImage& image, const vector<ofVec3f>& vertexs)
	{
		ofVec2f o = texMapper.map(vertexs[0]);
		ofVec2f d = texMapper.map(vertexs[2]);
		texture.draw(image, o.x, o.y, 0.0f, d.x - o.x, d.y - o.y);
		redraw();
	}

	void Canvas::endAllTraces()
	{
		touchPoints.clear();
	}

	void Canvas::touchEvent(const DataTouch& touchPoint)
	{
		assert(touchPoint.getPolygon()->getId() == polygonId);
		ofVec3f pto = touchPoint.getTouchPoint();
		ofVec3f mapped(texMapper.map(pto));
		int id = touchPoint.getId();
		map<int, Trace*>::iterator t = traces.find(id);
		switch (touchPoint.getType())
		{
		case kTouchTypeStarted:
			touchPoints[id] = touchPoint;
			traces[id] = new Trace(mapped, foreColor);
			break;
		case kTouchTypeHolding:
			if(pto.distanceSquared(touchPoints[id].getTouchPoint()) > 0.0001)
			{
				touchPoints[id] = touchPoint;
				if (t != traces.end())
				{
					t->second->update(mapped);
					redraw();
				}
			}
			break;
		case kTouchTypeReleased:
			touchPoints.erase(id);
			break;
		}
		
	}
}
