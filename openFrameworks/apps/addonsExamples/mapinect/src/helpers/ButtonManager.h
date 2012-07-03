#ifndef BUTTON_MANAGER_H__
#define BUTTON_MANAGER_H__

#include <map>
#include "DataTouch.h"
#include "IButtonManager.h"

namespace mapinect {

	class ButtonManager: public IButtonManager
	{
	public:
		ButtonManager();
		void draw();
		int addButton(const IButtonPtr& btn);
		void removeButton(int id);
		void fireButtonEvent(DataTouch touch);
	private:

		map<int,IButtonPtr> buttons;
	};
}

#endif	// BUTTON_MANAGER_H__
