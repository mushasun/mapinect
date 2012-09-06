#ifndef STORYCONSTANTS_H__
#define STORYCONSTANTS_H__

#include "ofVec3f.h"

namespace story
{
	struct Camera 
	{
	public:
		Camera(const ofVec3f& position, const ofVec3f& lookAt)
			: position(position), lookAt(lookAt) { }
		ofVec3f position;
		ofVec3f lookAt;
	};

	class StoryConstants
	{
	public:

		static float				SPOT_ROTATION_PERIOD_TIME;
		static float				HOUSE_GARDEN_1_TIME;
		static int					MENU_LIVE_TIME;
		static float				MENU_RADIUS;
		static int					CANT_CAMERAS;
		static vector<Camera>		CAMERAS;				

		static void					LoadStoryConstants();

	};
}

#endif	// STORYCONSTANTS_H__
