#include "PowerPlant.h"

#include "ofGraphicsUtils.h"
#include "ObjectButton.h"
#include "../StoryConstants.h"
#include <cmath>

namespace story
{
	ofImage* PowerPlant::txTop = NULL;
	ofImage* PowerPlant::txSide = NULL;
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
		working = true;
		this->btnManager = btnManager;

		//associateTextures();
		buildType = BuildType::kPowerPlant;
		/*Button in floor of box*/
		ObjectButton btnOnOff(object, kPolygonNameSideA, true, txSwitchOff, txSwitchOn,
								0.05,0.05, 0 ,0.04);

		btnManager->addButton(ObjectButtonPtr(new ObjectButton(btnOnOff)));
		buttonsId.push_back(btnOnOff.getId());
		actionsMap[btnOnOff.getId()] = POWER_SWITCH;
	}

	/* Events */
	/*-------------------------------------------------------------*/
	void PowerPlant::buttonEvent(const IButtonPtr& btn, bool released)
	{
		if(actionsMap.find(btn->getId()) == actionsMap.end())
			return; // no es un boton de la casa

		PowerPlantAction action = actionsMap[btn->getId()];
		switch(action)
		{
		case POWER_SWITCH:
				if(!released)
					if (working)
						{
							offSound->play();
						}
					else
						{
							onSound->play();
						}
					working = !working;
				break;
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

	/*-------------------------------------------------------------*/
	void PowerPlant::draw()
	{

	}


	/* Textures */
	/*-------------------------------------------------------------*/
	void PowerPlant::loadTextures()
	{
			txTop = new ofImage("data/texturas/house/top.jpg");
			txSide = new ofImage("data/texturas/house/Side.jpg");
			txSwitchOn = new ofImage("data/texturas/house/LightSwitchOn.jpg");
			txSwitchOff = new ofImage("data/texturas/house/LightSwitchOff.jpg");
	}

	/*-------------------------------------------------------------*/
	/*void PowerPlant::associateTextures()
	{
		textureTop = txTop;
		textureA = txSide;
		textureD = txSide;
	}*/

	/* Sounds */
	/*-------------------------------------------------------------*/
	void PowerPlant::loadSounds()
	{
		onSound = new ofSoundPlayer();
		onSound->loadSound("data/sonidos/house/ding.wav");
		offSound = new ofSoundPlayer();
		offSound->loadSound("data/sonidos/house/knock.wav");
	}
}
