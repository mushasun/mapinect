#include "PowerPlant.h"

#include "ofGraphicsUtils.h"
#include "ObjectButton.h"
#include "../StoryConstants.h"
#include <cmath>

namespace story
{
	ofImage* PowerPlant::txTop = NULL;
	ofImage* PowerPlant::txSideA = NULL;
	ofImage* PowerPlant::txSideB = NULL;
	ofImage* PowerPlant::txSwitchOn = NULL;
	ofImage* PowerPlant::txSwitchOff = NULL;
	ofSoundPlayer*	PowerPlant::onSound = NULL;
	ofSoundPlayer*	PowerPlant::offSound = NULL;	

	/*-------------------------------------------------------------*/
	void PowerPlant::setup()
	{
		loadSounds();
		loadTextures();
	}

	/*-------------------------------------------------------------*/
	PowerPlant::PowerPlant(const IObjectPtr& object, IButtonManager* btnManager):Box(object,btnManager)
	{
		working = false;
		this->btnManager = btnManager;

		//associateTextures();
		buildType = BuildType::kPowerPlant;
		/*Button in floor of box*/
		ObjectButton btnOnOff(object, kPolygonNameSideA, true, txSwitchOff, txSwitchOff,
								0.05,0.05, 0 ,0.04);

		btnManager->addButton(ObjectButtonPtr(new ObjectButton(btnOnOff)));
		buttonsId.push_back(btnOnOff.getId());
		actionsMap[btnOnOff.getId()] = POWER_SWITCH;

		associateTextures();
	}

	/* Events */
	/*-------------------------------------------------------------*/
	void PowerPlant::buttonEvent(const IButtonPtr& btn, bool released)
	{
		if(actionsMap.find(btn->getId()) == actionsMap.end())
			return; // no es un boton de la casa

		cout << "accion de power" << endl;
		PowerPlantAction action = actionsMap[btn->getId()];
		switch(action)
		{
			case POWER_SWITCH:
				if(!released)
				{
					if (working)
					{
						btnManager->setPressed(txSwitchOff, btn->getId());
						btnManager->setIdle(txSwitchOff, btn->getId());
						offSound->play();
						onSound->stop();
					}
					else
					{
						onSound->play();
						btnManager->setPressed(txSwitchOn, btn->getId());
						btnManager->setIdle(txSwitchOn, btn->getId());
					}
					working = !working;
				}
				break;
			default:
				cout << "[NO action]" << endl;
		}
	}

	/*-------------------------------------------------------------*/
	void PowerPlant::objectEvent(const DataTouch& touchPoint, const BuildType& selection)
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
	void PowerPlant::update(float elapsedTime)
	{

	}



	/* Textures */
	/*-------------------------------------------------------------*/
	void PowerPlant::loadTextures()
	{
			txTop = new ofImage("data/texturas/power/top.jpg");
			txSideA = new ofImage("data/texturas/power/SideA.jpg");
			txSideB = new ofImage("data/texturas/power/SideB.jpg");
			txSwitchOn = new ofImage("data/texturas/power/btnon.jpg");
			txSwitchOff = new ofImage("data/texturas/power/btnOff.jpg");
	}

	/*-------------------------------------------------------------*/
	void PowerPlant::associateTextures()
	{
		textureTop = txTop;
		textureA = txSideA;
		textureB = txSideB;
		textureC = txSideB;
		textureD = txSideB;
	}

	/* Sounds */
	/*-------------------------------------------------------------*/
	void PowerPlant::loadSounds()
	{
		onSound = new ofSoundPlayer();
		onSound->loadSound("data/sonidos/power/power-up.wav");
		onSound->setLoop(true);
		onSound->setVolume(0.5);
		offSound = new ofSoundPlayer();
		offSound->loadSound("data/sonidos/power/generator-off.wav");
	}
}
