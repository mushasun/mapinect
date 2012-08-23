#include "Menu.h"

#include "ofGraphicsUtils.h"
#include "ObjectButton.h"
#include "SimpleButton.h"

#include "PownConstants.h"
#include "SoundManager.h"

namespace pown
{
	ofImage** Menu::buttonTextures = NULL;
	const int kTexturesPerButton = 2;

	void Menu::setup(IButtonManager* btnManager)
	{
		this->btnManager = btnManager;

		loadSounds();
		loadTextures();
	}

	Menu::Menu()
		: active(false), timeMenuShown(0.0f)
	{
	}

	Menu::~Menu()
	{
		if (buttonTextures != NULL)
		{
			for (int i = 0; i < kTexturesPerButton * BUTTON_COUNT; i++)
			{
				delete buttonTextures[i];
			}
			delete buttonTextures;
			buttonTextures = NULL;
		}
	}

	void Menu::buttonEvent(const IButtonPtr& btn, bool released)
	{
		if (active && released) 
		{
			if(actions.find(btn->getId()) == actions.end())
				return; // the button isn't part of this menu

			MenuAction action = actions[btn->getId()];
			
			static int program = 0;

			cout << action << endl;
			switch(action)
			{
				case PROGRAM:
					program = (program + 1) % PROGRAMS;
					SoundManager::setProgram(program);
					removeMenu();
					break;
				case STAMP:
					
					break;
			}
		}
		else if (active)
		{
			timeMenuShown = 0.0f;
		}

	}

	void Menu::objectEvent(const IObjectPtr& object, const DataTouch& touchPoint)
	{
		if (timeMenuShown <= 0.0f && !active)
		{
			active = true;
			timeMenuShown = 0.0f;
			const float sideLength = PownConstants::MENU_BUTTON_SIZE;
			const float elevation = -0.005f;

			// create the buttons
			ofVec3f height = ofVec3f(0.0f, elevation, sideLength);
			ofVec3f width = ofVec3f(sideLength, elevation, 0.0f);
			const ofVec3f separation(sideLength * 1.2f, 0.0f, 0.0f);
			ofVec3f begin = touchPoint.getTouchPoint() - (separation * (float)BUTTON_COUNT * 0.5f);

			for (int i = 0; i < BUTTON_COUNT; i++)
			{
				vector<ofVec3f> button_vertex;
				button_vertex.push_back(begin + height);
				button_vertex.push_back(begin - height);
				button_vertex.push_back(begin - height + width);
				button_vertex.push_back(begin + height + width);

				Polygon3D area(button_vertex);
				SimpleButton *button = new SimpleButton(
					area,
					buttonTextures[i * kTexturesPerButton],
					buttonTextures[i * kTexturesPerButton + 1]);
				actions[button->getId()] = (MenuAction)i;
				IButtonPtr buttonPtr(button);
				btnManager->addButton(buttonPtr);
				begin += separation;
			}
		}
	}

	void Menu::update(float elapsedTime)
	{
		if (timeMenuShown >= 0)
		{
			timeMenuShown += elapsedTime;
			if (timeMenuShown > PownConstants::MENU_SHOW_TIME)
			{
				timeMenuShown = -1.0f;
				if (active) 
				{
					removeMenu();
				}
			}
		}
	}

	void Menu::draw() const
	{
		
	}

	void Menu::removeMenu()
	{
		for(map<int,MenuAction>::iterator it = actions.begin(); it != actions.end(); ++it)
		{
			btnManager->removeButton(it->first);
		}
		actions.clear();
		active = false;
	}

	void Menu::loadTextures()
	{
		if (buttonTextures == NULL)
		{
			buttonTextures = new ofImage* [BUTTON_COUNT * kTexturesPerButton];
			buttonTextures[0] = new ofImage("data/song.jpg");
			buttonTextures[1] = new ofImage("data/songOn.jpg");
			buttonTextures[2] = new ofImage("data/stamp.jpg");
			buttonTextures[3] = new ofImage("data/stampOn.jpg");
		}
	}

	void Menu::loadSounds()
	{
	}
}
