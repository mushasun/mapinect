#include "House.h"

#include "ofGraphicsUtils.h"
#include "ObjectButton.h"
#include "StoryConstants.h"
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
	ofImage* House::txHouseGarden1 = NULL;
	ofImage* House::txHouseGarden2 = NULL;
	ofImage* House::txHouseGarden3 = NULL;
	ofFbo 	 House::gardenFbo;
	ofSoundPlayer*	House::ding = NULL;
	ofSoundPlayer*	House::knock = NULL;
	ofSoundPlayer*	House::click = NULL;
	ofSoundPlayer*	House::call = NULL;
	ofSoundPlayer*	House::water = NULL;
	

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
		connected_to_energy = false;
		connected_to_water = false;
		lastWateringInSeconds = 0;
		isWatering = false;
		this->btnManager = btnManager;

		assosiateTextures();
		buildType = BuildType::kHouse;
		/*Button in floor of box*/
		ObjectButton btnDoorbell(object, kPolygonNameSideA, true, txHouseDoorBellOff, txHouseDoorBellOn,
								0.05,0.05, 0 ,0.04);
		
		
		
		/*Calculo jardin*/
		Polygon3D polB = object->getPolygon(kPolygonNameSideB)->getMathModel();
		float widthB = fabs((polB.getVertexs().at(1) - polB.getVertexs().at(2)).length());
		float heightB = fabs((polB.getVertexs().at(1) - polB.getVertexs().at(0)).length());
		ObjectButton btnGarden(object, kPolygonNameSideB, true, txHouseGarden1, txHouseGarden2, 
								0.05,widthB, 0 , 0.02);
		gardenBtnId = btnGarden.getId();

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
		btnManager->addButton(ObjectButtonPtr(new ObjectButton(btnGarden)));
		buttonsId.push_back(btnGarden.getId());
		actionsMap[btnGarden.getId()] = GARDEN;

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
			case GARDEN:
				if (connected_to_water)
				{
					isWatering = !released;
					if(isWatering)
						water->setPaused(false);
					else
						water->setPaused(true);
					break;
				}
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
				case BuildType::kWaterPlant:
					connected_to_water = true;
					break;
				case BuildType::kPowerPlant:
					//dejar este case al final
					connected_to_energy = true;
					ObjectButton btnLightSwitch(object, kPolygonNameSideB, true, txLightSwitchOff, txLightSwitchOn,
								0.05,0.05, 0 ,0.04);
					buttonsId.push_back(btnLightSwitch.getId());
					actionsMap[btnLightSwitch.getId()] = CONNECT;
					break;
			}
		}
	}

	/*-------------------------------------------------------------*/
	void House::update(float elapsedTime)
	{
		if(isWatering)
			lastWateringInSeconds = max(lastWateringInSeconds - (elapsedTime*4), 0.0f);
		else
			lastWateringInSeconds = min(lastWateringInSeconds + elapsedTime, StoryConstants::HOUSE_GARDEN_1_TIME*4);
	}

	/*-------------------------------------------------------------*/
	void House::draw()
	{
		Box::draw();
		vector<ofVec3f> gardenVexs = btnManager->getVertexs(gardenBtnId);
		if(gardenVexs.size() > 0)
		{
			float factor1 = min((lastWateringInSeconds / StoryConstants::HOUSE_GARDEN_1_TIME)*255, 255.0f);
			float factor2 = min((lastWateringInSeconds / (StoryConstants::HOUSE_GARDEN_1_TIME*4))*255, 255.0f);
			
			gardenFbo.begin();
			glDisable(GL_DEPTH_TEST);
			ofSetColor(255,255,255,255);
			txHouseGarden1->draw(0,0);
			ofSetColor(255,255,255,factor1);
			txHouseGarden2->draw(0,0);
			ofSetColor(255,255,255,factor2);
			txHouseGarden3->draw(0,0);
			glEnable(GL_DEPTH_TEST);
			gardenFbo.end();

			ofSetColor(255,255,255,255);
			gardenFbo.getTextureReference().bind();
			glBegin(GL_QUADS);  
				glTexCoord2f(0, 0);											glVertex3f(gardenVexs[0].x, gardenVexs[0].y, gardenVexs[0].z);  
				glTexCoord2f(gardenFbo.getWidth(), 0);						glVertex3f(gardenVexs[1].x, gardenVexs[1].y, gardenVexs[1].z);  
				glTexCoord2f(gardenFbo.getWidth(), gardenFbo.getHeight());	glVertex3f(gardenVexs[2].x, gardenVexs[2].y, gardenVexs[2].z); 
				glTexCoord2f(0, gardenFbo.getHeight());						glVertex3f(gardenVexs[3].x, gardenVexs[3].y, gardenVexs[3].z);   
			glEnd();
			gardenFbo.getTextureReference().unbind();
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
			txHouseGarden1 = new ofImage("data/texturas/house/garden1.jpg");
			txHouseGarden2 = new ofImage("data/texturas/house/garden2.jpg");
			txHouseGarden3 = new ofImage("data/texturas/house/garden3.jpg");
			gardenFbo.allocate(txHouseGarden1->getWidth(),txHouseGarden1->getHeight());
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
		water = new ofSoundPlayer();
		water->loadSound("data/sonidos/house/water.wav");
		water->play();
		water->setPaused(true);
		water->setLoop(true);
	}
}
