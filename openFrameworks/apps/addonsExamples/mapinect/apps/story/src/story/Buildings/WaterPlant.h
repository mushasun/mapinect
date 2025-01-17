#ifndef WATERPLANT_H__
#define WATERPLANT_H__

#include "IObject.h"
#include "IButton.h"
#include "../Box.h"
#include <map>
#include "ofImage.h"
#include "ofMain.h"


using namespace mapinect;

namespace story
{
	enum WaterPlantAction 
	{
		WATER_SWITCH,
	};

	class WaterPlant : public Box
	{
		public:
			WaterPlant(const IObjectPtr& object, IButtonManager* btnManager);
			
			static void				setup();
			
			virtual void			buttonEvent(const IButtonPtr& btn, bool released);
			virtual void			objectEvent(const DataTouch& touchPoint, const BuildType& selection);

			virtual void			update(float elapsedTime);

		private:
			static void				loadTextures();
			void					associateTextures();
			static void				loadSounds();

			//control
			bool					working;
			map<int, WaterPlantAction>	actionsMap;
			IButtonManager*			btnManager;

			//sounds
			static ofSoundPlayer*	onSound;
			static ofSoundPlayer*	offSound;
			
			//textures
			static ofImage*			txTop;
			static ofImage*			txSideA;
			static ofImage*			txSideB;
			static ofImage*			txSideC;
			static ofImage*			txSwitchOn;
			static ofImage*			txSwitchOff;

		
	};
}

#endif	// HOUSE_H__
