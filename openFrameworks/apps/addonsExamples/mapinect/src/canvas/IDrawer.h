#ifndef IDRAWER_H__
#define IDRAWER_H__

#include "ofxCairoTexture.h"

namespace mapinect
{
	class IDrawer
	{
	public:
		static IDrawer* SCreate(const ofVec2f&, const ofColor& color);
		virtual void update(const ofVec2f&) = 0;
		virtual void draw(ofxCairoTexture&) = 0;
	};
}

#endif	// IDRAWER_H__
