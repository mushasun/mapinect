#include "WaterPlant.h"

#include "ofGraphicsUtils.h"
#include "ObjectButton.h"
#include "../StoryConstants.h"
#include "../StoryStatus.h"
#include <cmath>

namespace story
{
	ofImage* WaterPlant::txTop = NULL;
	ofImage* WaterPlant::txSideA = NULL;
	ofImage* WaterPlant::txSideB = NULL;
	ofImage* WaterPlant::txSideC = NULL;
	ofImage* WaterPlant::txSwitchOn = NULL;
	ofImage* WaterPlant::txSwitchOff = NULL;
	ofSoundPlayer*	WaterPlant::onSound = NULL;
	ofSoundPlayer*	WaterPlant::offSound = NULL;	

	/*-------------------------------------------------------------*/
	void WaterPlant::setup()
	{
		loadSounds();
		loadTextures();
	}

	/*-------------------------------------------------------------*/
	WaterPlant::WaterPlant(const IObjectPtr& object, IButtonManager* btnManager):Box(object,btnManager)
	{
		working = false;
		this->btnManager = btnManager;

		//associateTextures();
		buildType = BuildType::kWaterPlant;
		/*Button in floor of box*/
		ObjectButton btnOnOff(object, kPolygonNameSideA, true, txSwitchOff, txSwitchOff,
								0.05, 0.05, 0, 0.04);

		btnManager->addButton(ObjectButtonPtr(new ObjectButton(btnOnOff)));
		buttonsId.push_back(btnOnOff.getId());
		actionsMap[btnOnOff.getId()] = WATER_SWITCH;

		associateTextures();
	}

	/* Events */
	/*-------------------------------------------------------------*/
	void WaterPlant::buttonEvent(const IButtonPtr& btn, bool released)
	{
		if(actionsMap.find(btn->getId()) == actionsMap.end())
			return; // no es un boton de la casa

		WaterPlantAction action = actionsMap[btn->getId()];
		switch(action)
		{
			case WATER_SWITCH:
				if(released)
				{
					working = !working;

					if (working)
					{
						btn->setPressed(txSwitchOn);
						btn->setIdle(txSwitchOn);
						onSound->play();
						StoryStatus::setProperty(WATERPLANT_ACTIVE,true);
					}
					else
					{
						btn->setPressed(txSwitchOff);
						btn->setIdle(txSwitchOff);
						offSound->play();
						onSound->stop();
						StoryStatus::setProperty(WATERPLANT_ACTIVE,false);
					}
				}
				break;
		}
	}

	/*-------------------------------------------------------------*/
	void WaterPlant::objectEvent(const DataTouch& touchPoint, const BuildType& selection)
	{
		if(selection != NULL)
		{
			switch(selection)
			{
				case BuildType::kHouse:
				/*cosas*/ 
				break;
			}
		}
	}

	/*-------------------------------------------------------------*/
	void WaterPlant::update(float elapsedTime)
	{
		Box::update(elapsedTime);
	}


	/* Textures */
	/*-------------------------------------------------------------*/
	void WaterPlant::loadTextures()
	{
			txTop = new ofImage("data/texturas/water/top.jpg");
			txSideA = new ofImage("data/texturas/water/SideA.jpg");
			txSideB = new ofImage("data/texturas/water/SideB.jpg");
			txSideC = new ofImage("data/texturas/water/SideC.jpg");
			txSwitchOn = new ofImage("data/texturas/water/on.png");
			txSwitchOff = new ofImage("data/texturas/water/off.png");
	}

	/*-------------------------------------------------------------*/
	void WaterPlant::associateTextures()
	{
		textureTop = txTop;
		textureA = txSideA;
		textureB = txSideB;
		textureC = txSideB;
		textureD = txSideC;
	}

	/* Sounds */
	/*-------------------------------------------------------------*/
	void WaterPlant::loadSounds()
	{
		onSound = new ofSoundPlayer();
		onSound->loadSound("data/sonidos/water/water_on.mp3");
		onSound->setLoop(true);
		onSound->setVolume(0.2);
		offSound = new ofSoundPlayer();
		offSound->loadSound("data/sonidos/water/water_off.wav");
	}
}
