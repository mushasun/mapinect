#ifndef IMAPPER_H__
#define IMAPPER_H__

#include "ofVec2f.h"
#include "ofVec3f.h"

namespace drawing
{
	class IMapper
	{
	public:
		virtual ofVec2f mapToTexture(const ofVec3f&) const = 0;
	};
}

#endif	// IMAPPER_H__
