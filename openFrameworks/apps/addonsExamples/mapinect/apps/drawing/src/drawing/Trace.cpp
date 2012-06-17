#include "Trace.h"

namespace drawing
{

	IDrawer* IDrawer::SCreate(IMapper* mapper, const DataTouch& touchPoint, int color)
	{
		return new Trace(mapper, touchPoint, color);
	}
	
	Trace::Trace(IMapper* mapper, const DataTouch& touchPoint, int color)
		: lastTouchPoint(touchPoint), color(color)
	{
		this->mapper = mapper;
	}

	Trace::~Trace()
	{
	}

	void Trace::update(const DataTouch& touchPoint)
	{
	}

	void Trace::draw(ofxCairoTexture& texture)
	{
	}

}
