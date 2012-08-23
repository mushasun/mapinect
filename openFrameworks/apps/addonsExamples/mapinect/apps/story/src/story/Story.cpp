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
		
		status = new StoryStatus();
		status->setup();

		menu.setup(status, btnManager);


		modeManager->disableObjectTracking();
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

		//Menu
		menu.update(elapsedTime);

		//Modo
		if (status->getProperty(ADDING_HOUSE) || status->getProperty(ADDING_POWERPLANT) || status->getProperty(ADDING_WATERPLANT))
		{
			modeManager->enableObjectTracking();
			modeManager->disableTouchTracking();
		}
		else if (status->getProperty(ADDING_RIVER) || status->getProperty(ADDING_STREET))
		{
			modeManager->disableObjectTracking();
			modeManager->enableTouchTracking();
		}

		//Buildings
		for (map<int, Box*>::const_iterator it = boxes.begin(); it != boxes.end(); ++it)
			it->second->update(elapsedTime);

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
			if (status->getProperty(ADDING_HOUSE))
			{
				House* h  = new House(object, btnManager);
				boxes.insert(pair<int, Box*>(object->getId(), h));
				status->setProperty(ADDING_HOUSE,false);
				modeManager->disableObjectTracking();
				modeManager->enableTouchTracking();
			}
			else if (status->getProperty(ADDING_POWERPLANT))
			{
				//poner aca la creacion de la power plant
			}
			else if (status->getProperty(ADDING_WATERPLANT))
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
		if (status->getProperty(ADDING_RIVER)||status->getProperty(ADDING_STREET))
		{
			if (firstTouchDone && firstTableTouch.distance(touchPoint.getTouchPoint())> 0.05)
			{
				if (status->getProperty(ADDING_RIVER))
				{
					river = River(firstTableTouch, touchPoint.getTouchPoint());					
				}
				if (status->getProperty(ADDING_STREET))
				{
					Road road = Road(firstTableTouch, touchPoint.getTouchPoint());
					btnManager->addButton(road.button);
					roads.push_back(road);
				}
				firstTouchDone = false;
				status->setProperty(ADDING_RIVER,false);
				status->setProperty(ADDING_STREET,false);
			}
			else
			{
				firstTouchDone = true;
				this->firstTableTouch = touchPoint.getTouchPoint();
			}
		}
		else
		{
			menu.objectEvent(object, touchPoint);
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

	void Story::buttonPressed(const IButtonPtr& btn, const DataTouch& touchPoint)
	{
		menu.buttonEvent(btn,false);
		for(map<int,Box*>::iterator it = boxes.begin(); it != boxes.end(); ++it)
			it->second->buttonEvent(btn, false);
	}

	void Story::buttonReleased(const IButtonPtr& btn, const DataTouch& touchPoint)
	{
		menu.buttonEvent(btn,true);
		for(map<int,Box*>::iterator it = boxes.begin(); it != boxes.end(); ++it)
		{
			it->second->buttonEvent(btn,true);
		}
	}

	void Story::pointTouched(const DataTouch& touchPoint)
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
	}

}
