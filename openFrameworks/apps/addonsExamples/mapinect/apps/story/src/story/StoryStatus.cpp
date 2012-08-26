#include "StoryStatus.h"

namespace story
{
	std::map<StoryStatusProperty, bool> StoryStatus::properties;

	void StoryStatus::setup()
	{
		properties[ADDING_STREET] = false;
		properties[ADDING_RIVER] = false;
		properties[ADDING_POWERPLANT] = false;
		properties[ADDING_WATERPLANT] = false;
		properties[ADDING_HOUSE] = false;
		properties[POWERPLANT_ACTIVE] = false;
		properties[WATERPLANT_ACTIVE] = false;
	}

	void StoryStatus::setProperty(StoryStatusProperty prop, bool val) 
	{ 
		properties[prop] = val; 
	}

	bool StoryStatus::getProperty(StoryStatusProperty prop) 
	{ 
		return properties[prop]; 
	}
	
}
