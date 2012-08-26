#ifndef SORYSTATUS_H__
#define SORYSTATUS_H__

#include <map>
#include "ofMain.h"

namespace story
{
	
	enum StoryStatusProperty{
		ADDING_STREET = 0,
		ADDING_RIVER,
		ADDING_POWERPLANT,
		ADDING_WATERPLANT,
		ADDING_HOUSE,
		POWERPLANT_ACTIVE,
		WATERPLANT_ACTIVE
	};
	
	class StoryStatus
	{
		private:
			static std::map<StoryStatusProperty, bool> properties;
		public:
		static void			setup();
		static void			setProperty(StoryStatusProperty prop, bool val);
		static bool			getProperty(StoryStatusProperty prop);
	};

}

#endif	// SORYSTATUS_H__
