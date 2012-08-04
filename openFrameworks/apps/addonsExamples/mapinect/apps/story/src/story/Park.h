#ifndef PARK_H__
#define PARK_H__

#include "IObject.h"
#include "ofImage.h"
#include "ofVec3f.h"

using namespace mapinect;

namespace story
{
	class Park
	{
	public:
		Park(const ofVec3f begin, const ofVec3f end);
		virtual ~Park();

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