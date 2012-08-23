#ifndef MENU_H__
#define MENU_H__

#include "IButtonManager.h"
#include "DataTouch.h"
#include "ofImage.h"

using namespace mapinect;

namespace pown
{
	enum MenuAction
	{
		PROGRAM = 0,
		STAMP,
		BUTTON_COUNT
	};

	class Menu
	{
	public:
		Menu();
		virtual ~Menu();

		void			setup(IButtonManager* btnManager);
		void			update(float elapsedTime);
		void			draw() const;
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

		static ofImage**		buttonTextures;
	};
}

#endif	// MENU_H__
