#include "StoryConstants.h"

#include "ofxXmlSettings.h"

#define STORY_CONFIG			"StoryConfig:"

namespace story
{

	float				StoryConstants::SPOT_BASE_RADIUS					= 0.05f;
	float				StoryConstants::SPOT_ROTATION_PERIOD_TIME			= 3.0f;

	void StoryConstants::LoadStoryConstants()
	{
		ofxXmlSettings XML;
		if(XML.loadFile("Story_Config.xml"))
		{
			SPOT_BASE_RADIUS				= XML.getValue(STORY_CONFIG "SPOT_BASE_RADIUS", SPOT_BASE_RADIUS);
			SPOT_ROTATION_PERIOD_TIME		= XML.getValue(STORY_CONFIG "SPOT_ROTATION_PERIOD_TIME", SPOT_ROTATION_PERIOD_TIME);
		}
	}

}