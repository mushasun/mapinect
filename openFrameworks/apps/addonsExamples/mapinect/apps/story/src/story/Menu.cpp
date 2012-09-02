#include "Menu.h"

#include "ofGraphicsUtils.h"
#include "ObjectButton.h"
#include "SimpleButton.h"
#include "StoryConstants.h"
#include <cmath>

namespace story
{
	ofImage* Menu::imgStreetButton = NULL;
	ofImage* Menu::imgRiverButton = NULL;
	ofImage* Menu::imgPowerPlantButton = NULL;
	ofImage* Menu::imgWaterPlantButton = NULL;
	ofImage* Menu::imgHouseButton = NULL;
	ofImage* Menu::imgStreetButtonOn = NULL;
	ofImage* Menu::imgRiverButtonOn = NULL;
	ofImage* Menu::imgPowerPlantButtonOn = NULL;
	ofImage* Menu::imgWaterPlantButtonOn = NULL;
	ofImage* Menu::imgHouseButtonOn = NULL;
	ofImage* Menu::imgFire = NULL;
	ofImage* Menu::imgFireOn = NULL;

	ofSoundPlayer*	Menu::ding = NULL;

	

	/*-------------------------------------------------------------*/
	void Menu::setup(IButtonManager* btnManager)
	{
		this->btnManager = btnManager;

		loadSounds();
		loadTextures();
	}

	/*-------------------------------------------------------------*/
	Menu::Menu()
	{
		active = false;
		timeMenuShown = 0;
	}

	/*-------------------------------------------------------------*/
	Menu::~Menu()
	{
		delete(imgStreetButton);
		delete(imgStreetButtonOn);
		delete(imgRiverButton);
		delete(imgRiverButtonOn);
		delete(imgPowerPlantButton);
		delete(imgPowerPlantButtonOn);
		delete(imgWaterPlantButton);
		delete(imgWaterPlantButtonOn);
		delete(imgHouseButton);
		delete(imgHouseButtonOn);
	}

	/* Events */
	/*-------------------------------------------------------------*/
	void Menu::buttonEvent(const IButtonPtr& btn, bool released)
	{
		if (active && released) 
		{
			if(actions.find(btn->getId()) == actions.end())
				return; // no es un boton del menu

			MenuAction action = actions[btn->getId()];
		
			switch(action)
			{
				case STREET:
					StoryStatus::setProperty(ADDING_STREET, true);
					break;
				case RIVER:
					StoryStatus::setProperty(ADDING_RIVER, true);
					break;
				case HOUSE:
					StoryStatus::setProperty(ADDING_HOUSE, true);
					break;
				case POWERPLANT:
					StoryStatus::setProperty(ADDING_POWERPLANT, true);
					break;
				case WATERPLANT:
					StoryStatus::setProperty(ADDING_WATERPLANT, true);
					break;
				case FIRE:
					StoryStatus::setProperty(WANT_TO_BURN, true);
					break;
			}
			
			ding->play();
			removeMenu();
		}
		else if (active)
		{
			timeMenuShown = 0;
		}

	}

	/*-------------------------------------------------------------*/
	void Menu::objectEvent(const IObjectPtr& object, const DataTouch& touchPoint)
	{
		if (timeMenuShown <= 0 && !active)
			{
				active = true;
				timeMenuShown = 0;			
				//dibujo el menu de cosas a construir
				ofVec3f begin = touchPoint.getTouchPoint();
				ofVec3f arriba = ofVec3f(0.f, 0.f, BUTTON_SIDE);
				ofVec3f costado = ofVec3f(BUTTON_SIDE, 0.f, 0.f);
				begin += costado;
				vector<ofVec3f> button_vertex;
				//primero casa
				button_vertex.push_back(begin + arriba);
				button_vertex.push_back(begin - arriba);
				button_vertex.push_back(begin - arriba + costado);
				button_vertex.push_back(begin + arriba + costado);
				Polygon3D area = Polygon3D(button_vertex);
				SimpleButton *houseButton = new SimpleButton(area, imgHouseButton, imgHouseButtonOn);
				actions[houseButton->getId()] = HOUSE;
				btnManager->addButton(IButtonPtr(houseButton));
				begin += costado;
				button_vertex.clear();

				//fire
				button_vertex.push_back(begin + arriba);
				button_vertex.push_back(begin - arriba);
				button_vertex.push_back(begin - arriba + costado);
				button_vertex.push_back(begin + arriba + costado);
				area = Polygon3D(button_vertex);
				SimpleButton *fireButton = new SimpleButton(area, imgFire, imgFireOn);
				actions[fireButton->getId()] = FIRE;
				btnManager->addButton(IButtonPtr(fireButton));
				begin += costado;
				button_vertex.clear();
				
				//central electrica
				button_vertex.push_back(begin + arriba);
				button_vertex.push_back(begin - arriba);
				button_vertex.push_back(begin - arriba + costado);
				button_vertex.push_back(begin + arriba + costado);
				area = Polygon3D(button_vertex);
				SimpleButton *powerPlantButton = new SimpleButton(area, imgPowerPlantButton, imgPowerPlantButtonOn);
				actions[powerPlantButton->getId()] = POWERPLANT;
				btnManager->addButton(IButtonPtr(powerPlantButton));
				begin += costado;
				button_vertex.clear();
				
				//water plant
				button_vertex.push_back(begin + arriba);
				button_vertex.push_back(begin - arriba);
				button_vertex.push_back(begin - arriba + costado);
				button_vertex.push_back(begin + arriba + costado);
				area = Polygon3D(button_vertex);
				SimpleButton *waterPlantButton = new SimpleButton(area, imgWaterPlantButton, imgWaterPlantButtonOn);
				actions[waterPlantButton->getId()] = WATERPLANT;
				btnManager->addButton(IButtonPtr(waterPlantButton));
				begin += costado;
				button_vertex.clear();

				//calle
				button_vertex.push_back(begin + arriba);
				button_vertex.push_back(begin - arriba);
				button_vertex.push_back(begin - arriba + costado);
				button_vertex.push_back(begin + arriba + costado);
				area = Polygon3D(button_vertex);
				SimpleButton *streetButton = new SimpleButton(area, imgStreetButton, imgStreetButtonOn);
				actions[streetButton->getId()] = STREET;
				btnManager->addButton(IButtonPtr(streetButton));
				begin += costado;
				button_vertex.clear();

				//river
				button_vertex.push_back(begin + arriba);
				button_vertex.push_back(begin - arriba);
				button_vertex.push_back(begin - arriba + costado);
				button_vertex.push_back(begin + arriba + costado);
				area = Polygon3D(button_vertex);
				SimpleButton *riverButton = new SimpleButton(area, imgRiverButton, imgRiverButtonOn);
				actions[riverButton->getId()] = RIVER;
				btnManager->addButton(IButtonPtr(riverButton));
				begin += costado;
				button_vertex.clear();
				//finished adding menu
			}
	}

	/*-------------------------------------------------------------*/
	void Menu::update(float elapsedTime)
	{
		if (timeMenuShown >= 0)
		{
			timeMenuShown += elapsedTime;
			if (timeMenuShown > StoryConstants::MENU_LIVE_TIME)
			{
				timeMenuShown = -1;
				if (active) 
				{
					removeMenu();
				}
			}
		}
	}

	/*-------------------------------------------------------------*/
	void Menu::draw()
	{
		
	}

	/*-------------------------------------------------------------*/
	void Menu::removeMenu()
	{
		for(map<int,MenuAction>::iterator it = actions.begin(); it != actions.end(); ++it)
		{
			btnManager->removeButton(it->first);
		}
		actions.clear();
		active = false;
	}

	/* Textures */
	/*-------------------------------------------------------------*/
	void Menu::loadTextures()
	{
		imgStreetButton = new ofImage("data/texturas/menu/road.jpg");
		imgRiverButton = new ofImage("data/texturas/menu/river.jpg");
		imgPowerPlantButton = new ofImage("data/texturas/menu/power.jpg");
		imgWaterPlantButton = new ofImage("data/texturas/menu/water.jpg");
		imgHouseButton = new ofImage("data/texturas/menu/house.jpg");

		imgStreetButtonOn = new ofImage("data/texturas/menu/roadOn.jpg");
		imgRiverButtonOn = new ofImage("data/texturas/menu/riverOn.jpg");
		imgPowerPlantButtonOn = new ofImage("data/texturas/menu/powerOn.jpg");
		imgWaterPlantButtonOn = new ofImage("data/texturas/menu/waterOn.jpg");
		imgHouseButtonOn = new ofImage("data/texturas/menu/houseOn.jpg");
		imgFire = new ofImage("data/texturas/menu/fire.png");
		imgFireOn = new ofImage("data/texturas/menu/fireOn.png");
	}

	

	/* Sounds */
	/*-------------------------------------------------------------*/
	void Menu::loadSounds()
	{
		ding = new ofSoundPlayer();
		ding->loadSound("data/sonidos/menu/ding.wav");

	}
}
