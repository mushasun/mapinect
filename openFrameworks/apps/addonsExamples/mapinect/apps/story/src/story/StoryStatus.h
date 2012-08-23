#ifndef SORYSTATUS_H__
#define SORYSTATUS_H__

#include <map>

namespace story
{
	
	enum StoryStatusProperty{
		ADDING_STREET = 0,
		ADDING_RIVER,
		ADDING_POWERPLANT,
		ADDING_WATERPLANT,
		ADDING_HOUSE
	};

	class StoryStatus
	{
		private:
			std::map<StoryStatusProperty, bool> properties;
		public:
			inline			StoryStatus(){};
			void			setup();
			inline void		setProperty(StoryStatusProperty prop, bool val) { properties[prop] = val; }
			inline bool		getProperty(StoryStatusProperty prop) { return properties[prop]; }
		
	};
}

#endif	// SORYSTATUS_H__
