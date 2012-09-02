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
		CONNECT,
		GARDEN
	};

	class House : public Box
	{
		public:
			House(const IObjectPtr& object, IButtonManager* btnManager);
			//virtual ~House(); 
			
			static void				setup();
			
			virtual void			buttonEvent(const IButtonPtr& btn, bool released);
			virtual void			objectEvent(const DataTouch& touchPoint, const BuildType& selection);

			virtual void			update(float elapsedTime);
			virtual void			draw();

		private:
			static void				loadTextures();
			void					associateTextures();
			static void				loadSounds();

			//control
			bool					lightsOn;
			bool					connected_to_energy;
			bool					connected_to_water;
			map<int,HouseAction>	actionsMap;
			float					lastWateringInSeconds;
			int						gardenBtnId;
			bool					isWatering;
			IButtonManager*			btnManager;

			float						timeToBurn;

			//sounds
			static ofSoundPlayer*	ding;
			static ofSoundPlayer*	knock;
			static ofSoundPlayer*	click;
			static ofSoundPlayer*	call;
			static ofSoundPlayer*	water;
			static ofSoundPlayer*	error;

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
			static ofImage*		txHouseGarden1;
			static ofImage*		txHouseGarden2;
			static ofImage*		txHouseGarden3;
			static ofFbo		gardenFbo;

		
	};
}

#endif	// HOUSE_H__
