#ifndef IDRAWER_H__
#define IDRAWER_H__

#include "IMapper.h"
#include "DataTouch.h"
#include "ofxCairoTexture.h"

using namespace mapinect;

namespace drawing
{
	class IDrawer
	{
	public:
		static IDrawer* SCreate(IMapper* mapper, const DataTouch&, int color);
		virtual void update(const DataTouch&) = 0;
		virtual void draw(ofxCairoTexture&) = 0;

	protected:
		IMapper*	mapper;
	};
}

#endif	// IDRAWER_H__
