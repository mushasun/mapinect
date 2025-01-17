#include "ButtonManager.h"
#include "EventManager.h"
#include "ObjectButton.h"
#include "ObjectButton.h"
#include "ofMain.h"

namespace mapinect {

	ButtonManager::ButtonManager()
	{
		EventManager::suscribe(this);
	}

	//// IButtonManager Implementation
	//--------------------------------------------------------------
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

	//--------------------------------------------------------------
	void ButtonManager::removeButton(int id)
	{
		if (buttons.find(id) != buttons.end())
		{
			buttons.erase(id);
		}
	}

	//--------------------------------------------------------------
	const IButtonPtr& ButtonManager::getButton(int id)
	{
		if (buttons.find(id) != buttons.end())
		{
			return buttons.find(id)->second;
		}
	}

	//--------------------------------------------------------------
	void ButtonManager::draw()
	{
		for (map<int, IButtonPtr>::const_iterator it = buttons.begin(); it != buttons.end(); ++it)
		{
			ofEnableAlphaBlending();
			BaseButton* button = dynamic_cast<BaseButton*>(it->second.get());
			button->draw();
			ofDisableAlphaBlending();
		}
	}

	//--------------------------------------------------------------
	/*void ButtonManager::fireButtonEvent(DataTouch touch)
	{
		for (map<int, IButtonPtr>::iterator iter = buttons.begin(); iter != buttons.end(); iter++) {
			
			ButtonEvent evnt = iter->second->updateTouchPoints(touch);
			switch(evnt)
			{
				case kButtonEventPressed: 
					EventManager::addEvent(MapinectEvent(kMapinectEventTypeButtonPressed, iter->second));
					break;
				case kButtonEventReleased:
					EventManager::addEvent(MapinectEvent(kMapinectEventTypeButtonReleased, iter->second));
					break;
				default:
					break;
			}
		}
	}*/

	//// INotification Implementation
	void ButtonManager::objectDetected(const IObjectPtr& object)
	{
		
	}

	//--------------------------------------------------------------
	void ButtonManager::objectUpdated(const IObjectPtr& object)
	{
		for (map<int, IButtonPtr>::const_iterator it = buttons.begin(); it != buttons.end(); ++it)
		{
			ObjectButton* oButton = dynamic_cast<ObjectButton*>(it->second.get());
			if(oButton != NULL)
				if(oButton->getObjectId() == object->getId())
					oButton->updateObject(object);

		}
	}

	//--------------------------------------------------------------
	void ButtonManager::objectLost(const IObjectPtr& object)
	{
		vector<int> toRemove;
		for (map<int, IButtonPtr>::const_iterator it = buttons.begin(); it != buttons.end(); ++it)
		{
			ObjectButton* oButton = dynamic_cast<ObjectButton*>(it->second.get());
			if(oButton != NULL)
				if(oButton->getObjectId() == object->getId())
					toRemove.push_back(oButton->getId());
		}
		for(int i = 0; i < toRemove.size(); i++)
			buttons.erase(toRemove.at(i));
	}

	//--------------------------------------------------------------
	void ButtonManager::objectMoved(const IObjectPtr& object, const DataMovement& movement)
	{
		for (map<int, IButtonPtr>::const_iterator it = buttons.begin(); it != buttons.end(); ++it)
		{
			ObjectButton* oButton = dynamic_cast<ObjectButton*>(it->second.get());
			if(oButton != NULL)
				if(oButton->getObjectId() == object->getId())
					oButton->updateObject(object);

		}
	}
	
	//--------------------------------------------------------------
	//void ButtonManager::objectTouched(const IObjectPtr& object, const DataTouch& touchPoint)
	//{
	//	//fireButtonEvent(touchPoint);
	//}

	//--------------------------------------------------------------
	void ButtonManager::objectTouchedPCM(const IObjectPtr& object, const DataTouch& touch)
	{
		bool btnTouched = false;
		for (map<int, IButtonPtr>::iterator iter = buttons.begin(); iter != buttons.end(); iter++) {
			
			ButtonEvent evnt = iter->second->updateTouchPoints(object, touch);
			switch(evnt)
			{
				case kButtonEventPressed: 
					EventManager::addEvent(MapinectEvent(kMapinectEventTypeButtonPressed, iter->second, touch));
					btnTouched = true;
					break;
				case kButtonEventReleased:
					EventManager::addEvent(MapinectEvent(kMapinectEventTypeButtonReleased, iter->second, touch));
					btnTouched = true;
					break;
				case kButtonEventHolding:
					btnTouched = true;
					break;
				case kButtonEventNotInButton:
					break;
			}
		}
		if(!btnTouched)
		{
			EventManager::addEvent(MapinectEvent(kMapinectEventTypeObjectTouched, object,touch));
		}
	}


}
