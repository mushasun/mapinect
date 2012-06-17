#ifndef TRACE_H__
#define TRACE_H__

#include "IDrawer.h"

#include "ofxCairoTexture.h"

using namespace mapinect;

namespace drawing
{
	class Trace : public IDrawer
	{
	public:
		Trace(IMapper*, const DataTouch&, int color);
		virtual ~Trace();

		void update(const DataTouch&);
		void draw(ofxCairoTexture&);

	private:
		DataTouch		lastTouchPoint;
		int				color;
	};
}

#endif	// TRACE_H__
