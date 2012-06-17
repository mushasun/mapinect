#ifndef TRACE_H__
#define TRACE_H__

#include "IDrawer.h"

#include "ofxCairoTexture.h"

namespace drawing
{
	class Trace : public IDrawer
	{
	public:
		Trace(const ofVec2f& startPoint, int color);
		virtual ~Trace();

		void update(const ofVec2f&);
		void draw(ofxCairoTexture&);

	private:
		ofVec2f			lastPoint;
		int				color;
		ofPolyline		polyline;
	};
}

#endif	// TRACE_H__
