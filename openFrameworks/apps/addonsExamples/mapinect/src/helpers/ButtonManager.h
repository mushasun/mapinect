#ifndef BUTTON_MANAGER_H__
#define BUTTON_MANAGER_H__

#include "IButtonManager.h"
#include <map>

namespace mapinect {

	class ButtonManager : public IButtonManager
	{
	public:
		ButtonManager();
//		void setup();
		void draw();
		int addButton(const IButtonPtr& btn);
		void removeButton(int id);
		void fireButtonEvent(DataTouch touch);
		inline void setListener(IApplication* app) { listener = app; }
	private:
		map<int,IButtonPtr> buttons;
		IApplication* listener;
	};
}

#endif	// BUTTON_MANAGER_H__
