#ifndef SORYSTATUS_H__
#define SORYSTATUS_H__

#include <map>
#include "ofMain.h"
#include "ofVec3f.h"
#include "IModeManager.h"

namespace story
{
	enum ofVec3fStoryStatusProperty{
		CENTROID_BURNING_HOUSE = 0,
	};
	
	enum StoryMode{
		STORY_ACTION_MODE = 0,
		STORY_MOVE_MODE,
		STORY_MOVE_AND_ACTION_MODE,
		STORY_ARM_MOVING,
		STORY_ARM_STOPPED
	};

	enum StoryStatusProperty{
		ADDING_STREET = 0,
		ADDING_RIVER,
		ADDING_POWERPLANT,
		ADDING_WATERPLANT,
		ADDING_HOUSE,
		POWERPLANT_ACTIVE,
		WATERPLANT_ACTIVE,
		FIREMAN_FINISHED,
		ALREADY_BURNING,
		WANT_TO_BURN,
		BURNING,
		SET_CAMERA_1,
		SET_CAMERA_2,
		SET_CAMERA_3,
	};

	enum IntStoryStatusProperty{
		ID_BURNING_HOUSE = 0,
	};


	
	class StoryStatus
	{
		private:
			static std::map<StoryStatusProperty, bool> properties;
			static mapinect::IModeManager* modeManager;
			static std::map<IntStoryStatusProperty, int> intProperties;
			static std::map<ofVec3fStoryStatusProperty, ofVec3f> ofVec3fProperties;

			static StoryMode currentMode;
		public:
			static void			setup(mapinect::IModeManager* modeManager);
			static void			setProperty(StoryStatusProperty prop, bool val);
			static void			setProperty(IntStoryStatusProperty prop, int val);
			static void			setProperty(ofVec3fStoryStatusProperty prop, ofVec3f val);
			static bool			getProperty(StoryStatusProperty prop);
			static int			getIntProperty(IntStoryStatusProperty prop);
			static ofVec3f		getofVec3fProperty(ofVec3fStoryStatusProperty prop);

			static void			setStoryMode(StoryMode mode);
			static StoryMode	getStoryMode();

	};

}

#endif	// SORYSTATUS_H__
