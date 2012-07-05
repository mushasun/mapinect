#include "BaseButton.h"
#include "pointutils.h"

namespace mapinect {
	void BaseButton::init()
	{
		id = btnIds;
		btnIds++;
		leaderTouch = -1;
		leaderChanged = false;
	}

	BaseButton::BaseButton(ofColor idle, ofColor pressed):
	idleColor(idle), pressedColor(pressed)
	{
		init();
		mode = kColor;
	}

	BaseButton::BaseButton(ofImage* idle, ofImage* pressed):
	texIdle(idle), texPressed(pressed)
	{
		init();
		mode = kBgImg;
	}
		
	ButtonEvent BaseButton::updateTouchPoints(const DataTouch& touch)
	{
		int newLeader = leaderTouch;
		ButtonEvent evnt = NO_CHANGE;
		if(isInTouch(touch))
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

					//cout << "erease: " << touch.getId() << " - " << newLeader << endl;
				}
			}
			else
			{
				contacts[touch.getId()] = touch;
				if(leaderTouch == -1)
					newLeader = touch.getId();

				//cout << "added: " << touch.getId() << " - " << newLeader << endl;

			}
				//contacts.insert(pair<int,DataTouch>(touch.getId(),touch));

			if(contacts.size() == 0)
				evnt = RELEASED;
			else if(contacts.size() == 1 &&
					touch.getType() == kTouchTypeStarted)
				evnt = PRESSED;
			else
				evnt = NO_CHANGE;
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

				if(contacts.size() == 0)
					evnt = RELEASED;
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