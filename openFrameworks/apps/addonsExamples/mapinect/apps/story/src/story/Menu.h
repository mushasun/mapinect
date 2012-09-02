#ifndef MENU_H__
#define MENU_H__

#include "ofImage.h"
#include "IButtonManager.h"
#include "DataTouch.h"
#include "StoryStatus.h"
#include "ofMain.h"

using namespace mapinect;

#define BUTTON_SIDE 0.05f

namespace story
{
	enum MenuAction
	{
		STREET,
		RIVER,
		HOUSE,
		POWERPLANT,
		WATERPLANT,
		FIRE,
	};
	class Menu
	{
	public:
		Menu();
		~Menu();

		void			setup(IButtonManager* btnManager);
		void			update(float elapsedTime);
		void			draw();
		void			buttonEvent(const IButtonPtr& btn, bool released);
		void			objectEvent(const IObjectPtr& object, const DataTouch& touchPoint);
	private:
		void					loadTextures();
		void					loadSounds();
		void					removeMenu();

		bool					active;
		map<int, MenuAction>	actions;
		float					timeMenuShown;
		IButtonManager*			btnManager;

		static ofImage*			imgStreetButton;
		static ofImage*			imgStreetButtonOn;
		static ofImage*			imgRiverButton;
		static ofImage*			imgRiverButtonOn;
		static ofImage*			imgPowerPlantButton;
		static ofImage*			imgPowerPlantButtonOn;
		static ofImage*			imgWaterPlantButton;
		static ofImage*			imgWaterPlantButtonOn;
		static ofImage*			imgHouseButton;
		static ofImage*			imgHouseButtonOn;
		static ofImage*			imgFire;
		static ofImage*			imgFireOn;

		static ofSoundPlayer*	ding;

	};
}

#endif	// MENU_H__
