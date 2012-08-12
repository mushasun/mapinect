#ifndef RIVER_H__
#define RIVER_H__

#include "IObject.h"
#include "ofImage.h"
#include "ofVec3f.h"

using namespace mapinect;

namespace story
{
	class River
	{
	public:
		River();
		River(const ofVec3f begin, const ofVec3f end);
		virtual ~River();

		void			update(float elapsedTime);
		void			draw();

	private:
		ofImage*		texture;
		ofVec3f			begin;
		ofVec3f			end;
		ofVec3f			normal;
		
	};
}

#endif	// ROAD_H__