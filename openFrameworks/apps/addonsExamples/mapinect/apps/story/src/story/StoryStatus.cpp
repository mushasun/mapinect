#include "StoryStatus.h"

namespace story
{
	std::map<StoryStatusProperty, bool> StoryStatus::properties;
	std::map<IntStoryStatusProperty, int> StoryStatus::intProperties;
	std::map<ofVec3fStoryStatusProperty, ofVec3f> StoryStatus::ofVec3fProperties;

	void StoryStatus::setup()
	{
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
	
	ofVec3f StoryStatus::getofVec3fProperty(ofVec3fStoryStatusProperty prop) 
	{ 
		return ofVec3fProperties[prop]; 
	}
}
