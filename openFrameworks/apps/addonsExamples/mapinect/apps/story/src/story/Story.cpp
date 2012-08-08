#include "Story.h"

#include "SimpleButton.h"
#include "ObjectButton.h"
#include "House.h"
#include "StoryConstants.h"

namespace story {
	
	//--------------------------------------------------------------
	Story::Story() {
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
			House* h  = new House(object, btnManager);
			boxes.insert(pair<int, Box*>(object->getId(), h));
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
		boxes.erase(boxes.find(object->getId()));
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
			if (!firstTouchDone)
			{
				firstTouchDone = true;
				this->firstTableTouch = touchPoint.getTouchPoint();
			}
			else
			{
				Road road = Road(firstTableTouch, touchPoint.getTouchPoint());
				roads.push_back(road);
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
				touchedIdx->second->objectEvent(touchPoint,selectedBoxIdx->second->getBuildType());
		}
	}

	void Story::buttonPressed(const IButtonPtr& btn)
	{
		for(map<int,Box*>::iterator it = boxes.begin(); it != boxes.end(); ++it)
		{
			it->second->buttonEvent(btn,false);
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
