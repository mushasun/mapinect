#include "Trace.h"

namespace mapinect
{

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
		polyline.lineTo(mappedPoint.x, mappedPoint.y);
		lastPoint = mappedPoint;
	}

	void Trace::draw(ofxCairoTexture& texture)
	{
		texture.setColor(color);
		texture.draw(polyline);
		
		polyline.clear();
		polyline.addVertex(lastPoint.x, lastPoint.y);
	}

}
