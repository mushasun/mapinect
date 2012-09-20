#include "StoryStatus.h"

namespace story
{
	std::map<StoryStatusProperty, bool> StoryStatus::properties;
	std::map<IntStoryStatusProperty, int> StoryStatus::intProperties;
	mapinect::IModeManager* StoryStatus::modeManager = NULL;
	std::map<ofVec3fStoryStatusProperty, ofVec3f> StoryStatus::ofVec3fProperties;
	StoryMode StoryStatus::currentMode = STORY_ACTION_MODE;		// Modo inicial?

	void StoryStatus::setup(mapinect::IModeManager* manager)
	{
		modeManager = manager;
		properties[ADDING_STREET] = false;
		properties[ADDING_RIVER] = false;
		properties[ADDING_POWERPLANT] = false;
		properties[ADDING_WATERPLANT] = false;
		properties[ADDING_HOUSE] = false;
		properties[POWERPLANT_ACTIVE] = false;
		properties[WATERPLANT_ACTIVE] = false;
		properties[FIREMAN_FINISHED] = false;
		properties[ALREADY_BURNING] = false;
		properties[BURNING] = false;
		properties[SET_CAMERA_1] = false;
		properties[SET_CAMERA_2] = false;
		properties[SET_CAMERA_3] = false;
		intProperties[ID_BURNING_HOUSE] = -1;
		ofVec3fProperties[CENTROID_BURNING_HOUSE] = ofVec3f(0.0, 0.0, 0.0);
	}

	void StoryStatus::setProperty(StoryStatusProperty prop, bool val) 
	{ 
		properties[prop] = val;
	}

	void StoryStatus::setProperty(IntStoryStatusProperty prop, int val) 
	{ 
		intProperties[prop] = val; 
	}

	void StoryStatus::setProperty(ofVec3fStoryStatusProperty prop, ofVec3f val)
	{
		ofVec3fProperties[prop] = val;
	}

	bool StoryStatus::getProperty(StoryStatusProperty prop) 
	{ 
		return properties[prop]; 
	}

	int StoryStatus::getIntProperty(IntStoryStatusProperty prop) 
	{ 
		return intProperties[prop]; 
	}

	void StoryStatus::setStoryMode(StoryMode mode)
	{
		StoryStatus::currentMode = mode;

		switch(mode)
		{
			case STORY_ACTION_MODE:
                StoryStatus::setProperty(ADDING_POWERPLANT, false);
                StoryStatus::setProperty(ADDING_WATERPLANT, false);
				StoryStatus::setProperty(ADDING_HOUSE,false);
				StoryStatus::setProperty(ADDING_RIVER,false);
				StoryStatus::setProperty(ADDING_STREET,false);
				modeManager->disableObjectTracking();
				modeManager->enableTouchTracking();
				break;
			case STORY_MOVE_MODE:
				StoryStatus::setProperty(ADDING_POWERPLANT, false);
                StoryStatus::setProperty(ADDING_WATERPLANT, false);
				StoryStatus::setProperty(ADDING_HOUSE,false);
				StoryStatus::setProperty(ADDING_RIVER,false);
				StoryStatus::setProperty(ADDING_STREET,false);
				modeManager->enableObjectTracking();
				modeManager->disableTouchTracking();
				break;
			case STORY_MOVE_AND_ACTION_MODE:
				StoryStatus::setProperty(ADDING_POWERPLANT, false);
                StoryStatus::setProperty(ADDING_WATERPLANT, false);
				StoryStatus::setProperty(ADDING_HOUSE,false);
				StoryStatus::setProperty(ADDING_RIVER,false);
				StoryStatus::setProperty(ADDING_STREET,false);
				modeManager->enableObjectTracking();
				modeManager->enableTouchTracking();
				break;
			case STORY_ARM_MOVING:
				StoryStatus::setProperty(ADDING_POWERPLANT, false);
                StoryStatus::setProperty(ADDING_WATERPLANT, false);
				StoryStatus::setProperty(ADDING_HOUSE,false);
				StoryStatus::setProperty(ADDING_RIVER,false);
				StoryStatus::setProperty(ADDING_STREET,false);
				modeManager->disableObjectTracking();
				modeManager->disableTouchTracking();
				break;
			case STORY_ARM_STOPPED:
				StoryStatus::setProperty(ADDING_POWERPLANT, false);
                StoryStatus::setProperty(ADDING_WATERPLANT, false);
				StoryStatus::setProperty(ADDING_HOUSE,false);
				StoryStatus::setProperty(ADDING_RIVER,false);
				StoryStatus::setProperty(ADDING_STREET,false);
				modeManager->enableObjectTracking();
				modeManager->disableTouchTracking();
				break;
		}
	}

	StoryMode StoryStatus::getStoryMode() 
	{
		return StoryStatus::currentMode;		
	}
	
	ofVec3f StoryStatus::getofVec3fProperty(ofVec3fStoryStatusProperty prop) 
	{ 
		return ofVec3fProperties[prop]; 
	}
}
