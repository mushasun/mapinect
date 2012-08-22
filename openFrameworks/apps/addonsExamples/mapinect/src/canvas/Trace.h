#ifndef TRACE_H__
#define TRACE_H__

#include "IDrawer.h"

#include "ofxCairoTexture.h"

namespace mapinect
{
	class Trace : public IDrawer
	{
	public:
		Trace(const ofVec2f& startPoint, const ofColor& color);
		virtual ~Trace();

		void update(const ofVec2f&);
		void draw(ofxCairoTexture&);

	private:
		ofVec2f			lastPoint;
		ofColor			color;
		ofPolyline		polyline;
	};
}

#endif	// TRACE_H__
