#include "Trace.h"

namespace drawing
{

	IDrawer* IDrawer::SCreate(const ofVec2f& startPoint, int color)
	{
		return new Trace(startPoint, color);
	}
	
	Trace::Trace(const ofVec2f& startPoint, int color)
		: lastPoint(startPoint), color(color)
	{
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
		texture.draw(polyline);
	}

}
