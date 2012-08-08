#include "Story.h"

#include "SimpleButton.h"
#include "ObjectButton.h"

namespace story {
	
	//--------------------------------------------------------------
	Story::Story() {

	}

	//--------------------------------------------------------------
	Story::~Story() {

	}

	//--------------------------------------------------------------
	void Story::setup() {

	}

	//--------------------------------------------------------------
	void Story::debugDraw()
	{

	}

	//--------------------------------------------------------------
	void Story::draw()
	{

	}

	//--------------------------------------------------------------
	void Story::update() 
	{

	}

	//--------------------------------------------------------------
	void Story::keyPressed(int key)
	{
	}

	//--------------------------------------------------------------
	void Story::objectDetected(const IObjectPtr& object)
	{

	}
		
	//--------------------------------------------------------------
	void Story::objectUpdated(const IObjectPtr& object)
	{
		Box* b = new Box(object);
		boxes.insert(pair<int, Box*>(1, b));
	}

	//--------------------------------------------------------------
	void Story::objectLost(const IObjectPtr& object)
	{

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
	}

	void Story::buttonPressed(const IButtonPtr& btn)
	{
	}

	void Story::buttonReleased(const IButtonPtr& btn)
	{
	}
}
