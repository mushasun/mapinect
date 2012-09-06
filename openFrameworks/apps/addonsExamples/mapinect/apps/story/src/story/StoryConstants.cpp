#include "StoryConstants.h"

#include "ofxXmlSettings.h"

#define STORY_CONFIG			"StoryConfig:"

namespace story
{

	float				StoryConstants::SPOT_ROTATION_PERIOD_TIME			= 3.0f;
	float				StoryConstants::HOUSE_GARDEN_1_TIME					= 20.0f;
	int					StoryConstants::MENU_LIVE_TIME						= 10;
	float				StoryConstants::MENU_RADIUS							= 0.08;

	int					StoryConstants::CANT_CAMERAS						= 0;
	vector<Camera>		StoryConstants::CAMERAS;

	void StoryConstants::LoadStoryConstants()
	{
		ofxXmlSettings XML;
		float posX;

		if(XML.loadFile("Story_Config.xml"))
		{
			XML.pushTag("StoryConfig");

			SPOT_ROTATION_PERIOD_TIME		= XML.getValue("SPOT_ROTATION_PERIOD_TIME", SPOT_ROTATION_PERIOD_TIME);
			HOUSE_GARDEN_1_TIME				= XML.getValue("HOUSE_GARDEN_1_TIME", HOUSE_GARDEN_1_TIME);
			MENU_LIVE_TIME					= XML.getValue("MENU_LIVE_TIME", MENU_LIVE_TIME);
			MENU_RADIUS						= XML.getValue("MENU_RADIUS", MENU_RADIUS);
			
			CANT_CAMERAS					= XML.getValue("CANT_CAMERAS",0);

			// Cargar posicion y vector look at para las diferentes "cámaras"
			float posX, posY, posZ, lookAtX, lookAtY, lookAtZ;

			for(int i = 0; i < CANT_CAMERAS; i++) 
			{
				// Cargar los datos de cada cámara
				XML.pushTag("CAMERA",i);

				posX						= XML.getValue("POSITION:X", 0.0);
				posY						= XML.getValue("POSITION:Y", 0.0);
				posZ						= XML.getValue("POSITION:Z", 0.0);

				lookAtX						= XML.getValue("LOOKAT:X",0.0);
				lookAtY						= XML.getValue("LOOKAT:Y",0.0);
				lookAtZ						= XML.getValue("LOOKAT:Z",0.1);

				XML.popTag();

				Camera camera(ofVec3f(posX,posY,posZ), ofVec3f(lookAtX,lookAtY,lookAtZ));
				CAMERAS.push_back(camera);

			}


		}
	}

}