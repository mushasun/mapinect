#include "Story.h"

#include "ObjectButton.h"
#include "House.h"
#include "River.h"
#include "Road.h"
#include "StoryConstants.h"

namespace story {
	
	//--------------------------------------------------------------
	Story::Story()
	{
	}

	//--------------------------------------------------------------
	Story::~Story() {
		for (map<int, Box*>::iterator i = boxes.begin(); i != boxes.end(); i++) {
			delete i->second;
		}
		boxes.clear();
	}

	//--------------------------------------------------------------
	void Story::setup() {
		selectedBoxIdx = boxes.end();
		StoryConstants::LoadStoryConstants();
		Spot::setup();
		House::setup();
		addingStreet = false;
		addingRiver = false;
		imgStreetButton = ofImage("data/texturas/road/road.jpg");
		imgRiverButton = ofImage("data/texturas/river/river.jpg");
		imgPowerPlantButton = ofImage("data/texturas/power/sign.jpg");
		imgWaterPlantButton = ofImage("data/texturas/house/top.jpg");
		imgHouseButton = ofImage("data/texturas/house/top.jpg");
		streetButtonId = -1;
		riverButtonId = -1;
		powerPlantButtonId = -1;
		waterPlantButtonId = -1;
		houseButtonId = -1;
		modeManager->disableObjectTracking();
		addingHouse = false;
		addingPowePlant = false;
		addingRiver = false;
		addingStreet = false;
		addingWaterPlant = false;
		firstTouchDone = false;
	}

	//--------------------------------------------------------------
	void Story::debugDraw()
	{
		
	}

	//--------------------------------------------------------------
	void Story::draw()
	{
		for(map<int,Box*>::iterator it = boxes.begin(); it != boxes.end(); ++it)
			it->second->draw();
		if(spot.isActive())
			spot.draw();

		for (map<int, DataTouch>::const_iterator it = touchPoints.begin(); it != touchPoints.end(); ++it)
		{
			if (it->second.getType() == kTouchTypeStarted)
				ofSetHexColor(0xFF0000);
			else if (it->second.getType() == kTouchTypeHolding)
				ofSetHexColor(0x00FF00);
			else
				ofSetHexColor(0x0000FF);
			ofVec3f s = it->second.getTouchPoint();
			ofCircle(s.x, s.y, s.z, 0.01);
			
		}
	}

	void Story::removeMenu()
	{
		btnManager->removeButton(streetButtonId);
		btnManager->removeButton(riverButtonId);
		btnManager->removeButton(powerPlantButtonId);
		btnManager->removeButton(waterPlantButtonId);
		btnManager->removeButton(houseButtonId);
		streetButtonId = -1;
		riverButtonId = -1;
		powerPlantButtonId = -1;
		waterPlantButtonId = -1;
		houseButtonId = -1;

	}

	//--------------------------------------------------------------
	void Story::update(float elapsedTime) 
	{
		//spot
		if(spot.isActive())
			spot.update(elapsedTime);

		//touchpoints
		map<int, DataTouch> keep;
		for (map<int, DataTouch>::const_iterator it = touchPoints.begin(); it != touchPoints.end(); ++it)
		{
			if (it->second.getType() != kTouchTypeReleased)
				keep.insert(make_pair(it->first, it->second));
		}
		touchPoints = keep;
		if (timeMenuShown >= 0)
		{
			timeMenuShown += elapsedTime;
			if (timeMenuShown > 10)
			{
				timeMenuShown = -1;
				if (streetButtonId > 0) //con saber que existe uno me alcanza
				{
					removeMenu();
				}
			}
		}
	}

	//--------------------------------------------------------------
	void Story::keyPressed(int key)
	{
	}

	//--------------------------------------------------------------
	void Story::objectDetected(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			floor = object;
		}
		else
		{
			if (addingHouse)
			{
				House* h  = new House(object, btnManager);
				boxes.insert(pair<int, Box*>(object->getId(), h));
				addingHouse = false;
				modeManager->disableObjectTracking();
				modeManager->enableTouchTracking();
			}
			else if (addingPowePlant)
			{
				//poner aca la creacion de la power plant
			}
			else if (addingWaterPlant)
			{
				//poner aca la creacion de la water plant
			}
		}
	}
		
	//--------------------------------------------------------------
	void Story::objectUpdated(const IObjectPtr& object)
	{
		map<int,Box*>::iterator box = boxes.find(object->getId());
		if(box != boxes.end())
			box->second->updateModelObject(object);
		/*Box* b = new Box(object);
		boxes.insert(pair<int, Box*>(1, b));*/
	}

	//--------------------------------------------------------------
	void Story::objectLost(const IObjectPtr& object)
	{
		map<int,Box*>::iterator box = boxes.find(object->getId());
		if(box != boxes.end())
			boxes.erase(box);
	}

	//--------------------------------------------------------------
	void Story::objectMoved(const IObjectPtr& object, const DataMovement& movement)
	{

	}
	
	//--------------------------------------------------------------
	void Story::objectTouched(const IObjectPtr& object, const DataTouch& touchPoint)
	{
		//touchpoints
		map<int, DataTouch>::iterator it = touchPoints.find(touchPoint.getId());
		if (it == touchPoints.end())
		{
			//assert(touchPoint.getType() == kTouchTypeStarted);
			touchPoints.insert(make_pair(touchPoint.getId(), touchPoint));
		}
		else
		{
			it->second = touchPoint;
		}

		if (object->getId() == TABLE_ID)
		{
			//reseteo seleccion de objetos
			selectedBoxIdx = boxes.end(); 
			spot.setActive(false);

			touchTable(object, touchPoint);
		}
		else 
		{
			touchObject(object, touchPoint);
		}
	}

	void Story::touchTable(const IObjectPtr& object, const DataTouch& touchPoint)
	{
		if (addingRiver||addingStreet)
		{
			if (firstTouchDone && firstTableTouch.distance(touchPoint.getTouchPoint())> 0.05)
			{
				if (addingRiver)
				{
					river = River(firstTableTouch, touchPoint.getTouchPoint());					
				}
				if (addingStreet)
				{
					Road road = Road(firstTableTouch, touchPoint.getTouchPoint());
					btnManager->addButton(road.button);
					roads.push_back(road);
				}
				firstTouchDone = false;
				addingRiver = false;
				addingStreet = false;
			}
			else
			{
				firstTouchDone = true;
				this->firstTableTouch = touchPoint.getTouchPoint();
			}
		}
		else
		{
			if (timeMenuShown <= 0 && streetButtonId < 0)
			{
				timeMenuShown = 0;			
				//dibujo el menu de cosas a construir
				ofVec3f begin = touchPoint.getTouchPoint();
				ofVec3f arriba = ofVec3f(0.f, 0.f, BUTTON_SIDE);
				ofVec3f costado = ofVec3f(BUTTON_SIDE, 0.f, 0.f);
				vector<ofVec3f> button_vertex;
				//primero casa
				button_vertex.push_back(begin + arriba);
				button_vertex.push_back(begin - arriba);
				button_vertex.push_back(begin - arriba + costado);
				button_vertex.push_back(begin + arriba + costado);
				Polygon3D area = Polygon3D(button_vertex);
				SimpleButton *houseButton = new SimpleButton(area, &imgHouseButton, &imgHouseButton);
				houseButtonId = houseButton->getId();
				btnManager->addButton(IButtonPtr(houseButton));
				begin += costado;
				button_vertex.clear();
				//central electrica
				button_vertex.push_back(begin + arriba);
				button_vertex.push_back(begin - arriba);
				button_vertex.push_back(begin - arriba + costado);
				button_vertex.push_back(begin + arriba + costado);
				area = Polygon3D(button_vertex);
				SimpleButton *powerPlantButton = new SimpleButton(area, &imgPowerPlantButton, &imgPowerPlantButton);
				powerPlantButtonId = powerPlantButton->getId();
				btnManager->addButton(IButtonPtr(powerPlantButton));
				begin += costado;
				button_vertex.clear();
				//water plant
				button_vertex.push_back(begin + arriba);
				button_vertex.push_back(begin - arriba);
				button_vertex.push_back(begin - arriba + costado);
				button_vertex.push_back(begin + arriba + costado);
				area = Polygon3D(button_vertex);
				SimpleButton *waterPlantButton = new SimpleButton(area, &imgWaterPlantButton, &imgWaterPlantButton);
				waterPlantButtonId = waterPlantButton->getId();
				btnManager->addButton(IButtonPtr(waterPlantButton));
				begin += costado;
				button_vertex.clear();
				//calle
				button_vertex.push_back(begin + arriba);
				button_vertex.push_back(begin - arriba);
				button_vertex.push_back(begin - arriba + costado);
				button_vertex.push_back(begin + arriba + costado);
				area = Polygon3D(button_vertex);
				SimpleButton *streetButton = new SimpleButton(area, &imgStreetButton, &imgStreetButton);
				streetButtonId = streetButton->getId();
				btnManager->addButton(IButtonPtr(streetButton));
				begin += costado;
				button_vertex.clear();
				//river
				button_vertex.push_back(begin + arriba);
				button_vertex.push_back(begin - arriba);
				button_vertex.push_back(begin - arriba + costado);
				button_vertex.push_back(begin + arriba + costado);
				area = Polygon3D(button_vertex);
				SimpleButton *riverButton = new SimpleButton(area, &imgRiverButton, &imgRiverButton);
				riverButtonId = riverButton->getId();
				btnManager->addButton(IButtonPtr(riverButton));
				begin += costado;
				button_vertex.clear();
				//finished adding menu
			}
		}
	}

	void Story::touchObject(const IObjectPtr& object, const DataTouch& touchPoint)
	{
		map<int,Box*>::iterator touchedIdx = boxes.find(object->getId());
		if(touchedIdx != boxes.end())
		{
			if(selectedBoxIdx == boxes.end())
			{
				selectedBoxIdx = touchedIdx;
				ofVec3f spotCenter = floor->getPolygons()[0]->getMathModel().getPlane().project(object->getCenter());
				spotCenter.y -= 0.001f;
				spot.setPosition(spotCenter);
			}
			else
			{
				touchedIdx->second->objectEvent(touchPoint,selectedBoxIdx->second->getBuildType());
			}
		}
	}

	void Story::buttonPressed(const IButtonPtr& btn)
	{
		if (streetButtonId > 0) //con saber que existe uno me alcanza
		{
			addingStreet = (btn->getId() == streetButtonId);
			addingRiver = (btn->getId() == riverButtonId);
			addingPowePlant = (btn->getId() == powerPlantButtonId);
			addingWaterPlant = (btn->getId() == waterPlantButtonId);
			addingHouse = (btn->getId() == houseButtonId);
			if (addingHouse || addingPowePlant || addingWaterPlant)
			{
				modeManager->enableObjectTracking();
				modeManager->disableTouchTracking();
			}
			else if (addingRiver || addingStreet)
			{
				modeManager->disableObjectTracking();
				modeManager->enableTouchTracking();
			}
			removeMenu();
		}
		else
		{
			for(map<int,Box*>::iterator it = boxes.begin(); it != boxes.end(); ++it)
			it->second->buttonEvent(btn, false);
		}
	}

	void Story::buttonReleased(const IButtonPtr& btn)
	{
		for(map<int,Box*>::iterator it = boxes.begin(); it != boxes.end(); ++it)
		{
			it->second->buttonEvent(btn,true);
		}
	}
}
