#include "BaseButton.h"

#include "pointUtils.h"

namespace mapinect {
	
	static int btnIds = 0;

	BaseButton::BaseButton()
	{
	}

	void BaseButton::init()
	{
		id = btnIds;
		btnIds++;
		leaderTouch = -1;
		leaderChanged = false;
		zIndex = 1;
		touching = false;
	}

	BaseButton::BaseButton(const ofColor& idle, const ofColor& pressed)
		: idleColor(idle), pressedColor(pressed)
	{
		init();
		mode = kButtonDrawModePlain;
	}

	BaseButton::BaseButton(ofImage* idle, ofImage* pressed)
		: idleTexture(idle), pressedTexture(pressed)
	{
		init();
		mode = kButtonDrawModeTextured;
	}
		
	ButtonEvent BaseButton::updateTouchPoints(const IObjectPtr& object, const DataTouch& touch)
	{
		int newLeader = leaderTouch;
		ButtonEvent evnt = kButtonEventNotInButton;
		if(isInTouch(object, touch))
		{
			if(touch.getType() == kTouchTypeReleased &&
				contacts.size() > 0)
			{
				contacts.erase(touch.getId());
				if(leaderTouch == touch.getId())
				{
					if(contacts.size() == 0)
						newLeader = -1;
					else
						newLeader = contacts.begin()->second.getId();
				}
			}
			else
			{
				contacts[touch.getId()] = touch;
				if(leaderTouch == -1)
					newLeader = touch.getId();
			}

			if(contacts.size() == 0)
			{
				evnt = kButtonEventReleased;
				touching = false;
			}
			else if(contacts.size() == 1 &&
					(touch.getType() == kTouchTypeStarted || touch.getType() == kTouchTypeHolding) &&
					touching == false)
			{
				evnt = kButtonEventPressed;
				touching = true;
			}
			else
				evnt = kButtonEventHolding;
		}
		else
		{
			map<int,DataTouch>::iterator contact = contacts.find(touch.getId());
			if(contact != contacts.end())
			{
				contacts.erase(touch.getId());
				if(leaderTouch == touch.getId())
				{
					if(contacts.size() == 0)
						newLeader = -1;
					else
						newLeader = contacts.begin()->second.getId();
				}

				//cout << "erease: " << touch.getId() << " - " << newLeader << endl;

				/*if(contacts.size() == 0)
					evnt = kButtonEventReleased;*/
				evnt = kButtonEventNotInButton;
			}
		}

		if(newLeader != leaderTouch)
		{
			leaderTouch = newLeader;
			leaderChanged = true;
			//cout << "Leader changed : " << leaderChanged << "- " << leaderTouch << endl;
		}
		else
			leaderChanged = false;

		
		//cout << "Leader: " << leaderTouch << endl;
		return evnt;
	}
}