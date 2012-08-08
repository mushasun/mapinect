#include "House.h"

#include "ofGraphicsUtils.h"
#include "ObjectButton.h"
#include "ObjectButton.h"
#include <cmath>

namespace story
{
	ofImage* House::txHouseTop = NULL;
	ofImage* House::txHouseSide = NULL;
	ofImage* House::txHouseSideWindowOn = NULL;
	ofImage* House::txHouseSideWindowOff = NULL;
	ofImage* House::txHouseDoorBellOn = NULL;
	ofImage* House::txHouseDoorBellOff = NULL;
	ofImage* House::txLightSwitchOn = NULL;
	ofImage* House::txLightSwitchOff = NULL;
	ofImage* House::txHouseDoor = NULL;
	ofSoundPlayer*	House::ding = NULL;
	ofSoundPlayer*	House::knock = NULL;
	ofSoundPlayer*	House::click = NULL;
	ofSoundPlayer*	House::call = NULL;
	
	/*-------------------------------------------------------------*/
	void House::setup()
	{
		loadSounds();
		loadTextures();
	}

	/*-------------------------------------------------------------*/
	House::House(const IObjectPtr& object, IButtonManager* btnManager):Box(object,btnManager)
	{
		lightsOn = false;
		connected = false;
		assosiateTextures();
		buildType = BuildType::kHouse;
		/*Button in floor of box*/
		ObjectButton btnDoorbell(object, kPolygonNameSideA, true, txHouseDoorBellOff, txHouseDoorBellOn,
								0.05,0.05, 0 ,0.04);
		
		//Calculo de la puerta
		Polygon3D pol = object->getPolygon(kPolygonNameSideA)->getMathModel();
		float width = fabs((pol.getVertexs().at(1) - pol.getVertexs().at(2)).length());
		float height = fabs((pol.getVertexs().at(1) - pol.getVertexs().at(0)).length());
		float doorWidth = min(width, 0.05f);
		float doorHeight = min(height, 0.1f);
		float padding = (width/2) - doorWidth/2;


		ObjectButton btnDoor(object, kPolygonNameSideA, false, txHouseDoor, txHouseDoor,
								doorHeight,doorWidth,padding,0);

		btnManager->addButton(ObjectButtonPtr(new ObjectButton(btnDoorbell)));
		buttonsId.push_back(btnDoorbell.getId());
		actionsMap[btnDoorbell.getId()] = DOORBELL;
		btnManager->addButton(ObjectButtonPtr(new ObjectButton(btnDoor)));
		buttonsId.push_back(btnDoor.getId());
		actionsMap[btnDoor.getId()] = KNOCK;

	}

	/* Events */
	/*-------------------------------------------------------------*/
	void House::buttonEvent(const IButtonPtr& btn, bool released)
	{
		if(actionsMap.find(btn->getId()) == actionsMap.end())
			return; // no es un boton de la casa

		HouseAction action = actionsMap[btn->getId()];
		switch(action)
		{
			case DOORBELL:
				if(!released)
					ding->play();
				break;
			case LIGHT_SWITCH:
				if(released)
				{
					lightsOn = !lightsOn;
					click->play();
					assosiateTextures();
				}
				break;
			case KNOCK:
				if(!released)
					knock->play();
				break;
		}
	}

	/*-------------------------------------------------------------*/
	void House::objectEvent(const DataTouch& touchPoint, const BuildType& selection)
	{
		if(selection != NULL)
		{
			switch(selection)
			{
				case BuildType::kHouse:
					call->play();
					break;
				case BuildType::kPowerPlant:
					connected = true;
					ObjectButton btnLightSwitch(object, kPolygonNameSideB, true, txLightSwitchOff, txLightSwitchOn,
								0.05,0.05, 0 ,0.04);
					buttonsId.push_back(btnLightSwitch.getId());
					actionsMap[btnLightSwitch.getId()] = CONNECT;
					break;
			}
		}
	}


	/* Textures */
	/*-------------------------------------------------------------*/
	void House::loadTextures()
	{
			txHouseTop = new ofImage("data/texturas/house/top.jpg");
			txHouseSide = new ofImage("data/texturas/house/Side.jpg");
			txHouseSideWindowOn = new ofImage("data/texturas/house/SideWindowOn.jpg");
			txHouseSideWindowOff = new ofImage("data/texturas/house/SideWindowOff.jpg");
			txHouseDoorBellOn = new ofImage("data/texturas/house/DoorBellOn.jpg");
			txHouseDoorBellOff = new ofImage("data/texturas/house/DoorBellOff.jpg");
			txLightSwitchOn = new ofImage("data/texturas/house/LightSwitchOn.jpg");
			txLightSwitchOff = new ofImage("data/texturas/house/LightSwitchOff.jpg");
			txHouseDoor = new ofImage("data/texturas/house/Door.jpg");
	}

	/*-------------------------------------------------------------*/
	void House::assosiateTextures()
	{
		textureTop = txHouseTop;
		textureA = txHouseSide;
		textureB = lightsOn ? txHouseSideWindowOn : txHouseSideWindowOff;
		textureC = txHouseSide;
		textureD = txHouseSide;
	}

	/* Sounds */
	/*-------------------------------------------------------------*/
	void House::loadSounds()
	{
		ding = new ofSoundPlayer();
		ding->loadSound("data/sonidos/house/ding.wav");
		knock = new ofSoundPlayer();
		knock->loadSound("data/sonidos/house/knock.wav");
		click = new ofSoundPlayer();
		click->loadSound("data/sonidos/house/click.wav");
		call = new ofSoundPlayer();
		call->loadSound("data/sonidos/house/call.wav");
	}
}
