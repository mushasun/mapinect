#include "ButtonManager.h"
#include "EventManager.h"

namespace mapinect {

	ButtonManager::ButtonManager()
	{
	}

	void ButtonManager::draw()
	{
		for (map<int, IButtonPtr>::const_iterator it = buttons.begin(); it != buttons.end(); ++it)
			it->second->draw();
	}

	int ButtonManager::addButton(const IButtonPtr& btn)
	{
		if (buttons.find(btn->getId()) == buttons.end())
		{
			buttons[btn->getId()] = btn;
			return btn->getId();
		}
		else
			return -1;
	}

	void ButtonManager::removeButton(int id)
	{
		if (buttons.find(id) != buttons.end())
		{
			buttons.erase(id);
		}
	}

	void ButtonManager::fireButtonEvent(DataTouch touch)
	{
		for (map<int, IButtonPtr>::iterator iter = buttons.begin(); iter != buttons.end(); iter++) {
			
			ButtonEvent evnt = iter->second->updateTouchPoints(touch);
			switch(evnt)
			{
				case PRESSED: 
					EventManager::addEvent(MapinectEvent(kMapinectEventTypeButtonPressed, iter->second));
					//listener->buttonPressed(iter->second);
					break;
				case RELEASED:
					EventManager::addEvent(MapinectEvent(kMapinectEventTypeButtonReleased, iter->second));
					//listener->buttonReleased(iter->second);
					break;
				default:
					break;
			}
			//EventManager::fireEvents();
		}
	}
}
