#ifndef HOUSE_H__
#define HOUSE_H__

#include "IObject.h"
#include "IButton.h"
#include "Box.h"
#include <map>
#include "ofImage.h"
#include "ofMain.h"


using namespace mapinect;

namespace story
{
	enum HouseAction 
	{
		DOORBELL,
		LIGHT_SWITCH,
		KNOCK,
		CONNECT
	};

	class House : public Box
	{
		public:
			House(const IObjectPtr& object, IButtonManager* btnManager);
			//virtual ~House(); 
			
			static void		setup();
			
			virtual void buttonEvent(const IButtonPtr& btn, bool released);
			virtual void objectEvent(const DataTouch& touchPoint, const BuildType& selection);

		private:
			static void				loadTextures();
			void					assosiateTextures();
			static void				loadSounds();

			//control
			bool					lightsOn;
			bool					connected;
			map<int,HouseAction>	actionsMap;
			
			//sounds
			static ofSoundPlayer*	ding;
			static ofSoundPlayer*	knock;
			static ofSoundPlayer*	click;
			static ofSoundPlayer*	call;
			
			//textures
			static ofImage*		txHouseTop;
			static ofImage*		txHouseSide;
			static ofImage*		txHouseSideWindowOn;
			static ofImage*		txHouseSideWindowOff;

			static ofImage*		txHouseDoorBellOn;
			static ofImage*		txHouseDoorBellOff;
			static ofImage*		txLightSwitchOn;
			static ofImage*		txLightSwitchOff;
			static ofImage*		txHouseDoor;
		
	};
}

#endif	// HOUSE_H__
