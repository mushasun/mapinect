#include "StoryStatus.h"

namespace story
{
	void StoryStatus::setup()
	{
		properties[ADDING_STREET] = false;
		properties[ADDING_RIVER] = false;
		properties[ADDING_POWERPLANT] = false;
		properties[ADDING_WATERPLANT] = false;
		properties[ADDING_HOUSE] = false;
	}
	
}
