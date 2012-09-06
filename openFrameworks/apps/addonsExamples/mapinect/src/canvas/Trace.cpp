#include "Trace.h"

namespace mapinect
{

	IDrawer* IDrawer::SCreate(const ofVec2f& startPoint, const ofColor& color)
	{
		return new Trace(startPoint, color);
	}
	
	Trace::Trace(const ofVec2f& startPoint, const ofColor& color)
		: lastPoint(startPoint), color(color)
	{
		polyline.addVertex(startPoint.x, startPoint.y);
	}

	Trace::~Trace()
	{
	}

	void Trace::update(const ofVec2f& mappedPoint)
	{
		polyline.curveTo(mappedPoint.x, mappedPoint.y);
		lastPoint = mappedPoint;
	}

	void Trace::draw(ofxCairoTexture& texture)
	{
		texture.setColor(color);
		texture.draw(polyline);
	}

}
