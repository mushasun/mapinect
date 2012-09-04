#include "StoryConstants.h"

#include "ofxXmlSettings.h"

#define STORY_CONFIG			"StoryConfig:"

namespace story
{

	float				StoryConstants::SPOT_ROTATION_PERIOD_TIME			= 3.0f;
	float				StoryConstants::HOUSE_GARDEN_1_TIME					= 20.0f;
	int					StoryConstants::MENU_LIVE_TIME						= 10;
	float				StoryConstants::MENU_RADIUS							= 0.08;
	void StoryConstants::LoadStoryConstants()
	{
		ofxXmlSettings XML;
		if(XML.loadFile("Story_Config.xml"))
		{
			SPOT_ROTATION_PERIOD_TIME		= XML.getValue(STORY_CONFIG "SPOT_ROTATION_PERIOD_TIME", SPOT_ROTATION_PERIOD_TIME);
			HOUSE_GARDEN_1_TIME				= XML.getValue(STORY_CONFIG "HOUSE_GARDEN_1_TIME", HOUSE_GARDEN_1_TIME);
			MENU_LIVE_TIME					= XML.getValue(STORY_CONFIG "MENU_LIVE_TIME", MENU_LIVE_TIME);
			MENU_RADIUS						= XML.getValue(STORY_CONFIG "MENU_RADIUS", MENU_RADIUS);
		}
	}

}