#include "Story.h"

#include "ObjectButton.h"
#include "House.h"
#include "River.h"
#include "Road.h"
#include "StoryConstants.h"
#include "ofGraphicsUtils.h"
#include "Buildings/bomberos.h"


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
		Box::setup();
		House::setup();
		WaterPlant::setup();
		PowerPlant::setup();
		StoryStatus::setup(modeManager);
		Bomberos::setup();

		menu.setup(btnManager);

		modeManager->disableObjectTracking();
		firstTouchDone = false;
		river = NULL;

		previousMode = STORY_ACTION_MODE;	
		objectWasUpdated = false;
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

		if(river != NULL)
			river->draw();

		for (map<int, DataTouch>::const_iterator it = touchPoints.begin(); it != touchPoints.end(); ++it)
		{
			if (it->second.getType() == kTouchTypeStarted)
				ofSetHexColor(0xFF0000);
			else if (it->second.getType() == kTouchTypeHolding)
				ofSetHexColor(0x00FF00);
			else
				ofSetHexColor(0x0000FF);
			ofVec3f s = it->second.getTouchPoint();
			ofDrawCircle(s, 0.01);
		}
	}

	

	//--------------------------------------------------------------
	void Story::update(float elapsedTime) 
	{
		if (StoryStatus::getStoryMode() == STORY_ARM_STOPPED) {
			int MAX_WAITING_TIME = 2000;	// 2 segundos
			unsigned int elapsedTime = (unsigned int) (ofGetSystemTime() - startTime);
			if (objectWasUpdated || (elapsedTime > MAX_WAITING_TIME)) {
				// Tras actualizar los objetos, vuelve al modo en que estaba antes de mover el brazo
				StoryStatus::setStoryMode(previousMode);
				cout << "Volviendo al modo anterior luego de: " << elapsedTime << " ms" << endl;
				objectWasUpdated = false;
			}
		}

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
		if (StoryStatus::getProperty(ADDING_HOUSE) || StoryStatus::getProperty(ADDING_POWERPLANT) || StoryStatus::getProperty(ADDING_WATERPLANT))
		{
			modeManager->enableObjectTracking();
		}
		else if (StoryStatus::getProperty(ADDING_RIVER) || StoryStatus::getProperty(ADDING_STREET))
		{
			modeManager->disableObjectTracking();
		}
		//Buildings

		for (map<int, Box*>::const_iterator it = boxes.begin(); it != boxes.end(); ++it)
		{
			it->second->update(elapsedTime);
		}
		if (StoryStatus::getProperty(SET_CAMERA_1))
		{
			setCamera(1);
			StoryStatus::setProperty(SET_CAMERA_1,false);
		}
		if (StoryStatus::getProperty(SET_CAMERA_2))
		{
			setCamera(2);
			StoryStatus::setProperty(SET_CAMERA_2,false);
		}
		if (StoryStatus::getProperty(SET_CAMERA_3))
		{
			setCamera(3);
			StoryStatus::setProperty(SET_CAMERA_3,false);
		}

	}

	//--------------------------------------------------------------
	void Story::keyPressed(int key)
	{
		switch(key)
		{
			case 'c':
				StoryStatus::setStoryMode(STORY_ACTION_MODE);
				break;
			case 'm':
				StoryStatus::setStoryMode(STORY_MOVE_MODE);
				break;
			case '1':
				setCamera(1);
				break;
			case '2':
				setCamera(2);
				break;
			case '3':
				setCamera(3);
				break;
		}
	}

	//--------------------------------------------------------------
	void Story::objectDetected(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			tableUpdated(object);
		}
		else
		{
			if(StoryStatus::getProperty(BURNING))
			{
				Bomberos* b  = new Bomberos(object, btnManager);
				boxes.insert(pair<int, Box*>(object->getId(), b));
			}
			else if (StoryStatus::getProperty(ADDING_HOUSE))
			{
				House* h  = new House(object, btnManager);
				boxes.insert(pair<int, Box*>(object->getId(), h));
				//StoryStatus::setStoryMode(STORY_ACTION_MODE);
			}
			else if (StoryStatus::getProperty(ADDING_POWERPLANT))
			{
				PowerPlant* plant  = new PowerPlant(object, btnManager);
                boxes.insert(pair<int, Box*>(object->getId(), plant));
                //StoryStatus::setStoryMode(STORY_ACTION_MODE);
			}
			else if (StoryStatus::getProperty(ADDING_WATERPLANT))
			{
				WaterPlant* plant  = new WaterPlant(object, btnManager);
                boxes.insert(pair<int, Box*>(object->getId(), plant));
                //StoryStatus::setStoryMode(STORY_ACTION_MODE);
			}
		}
	}
		
	//--------------------------------------------------------------
	void Story::objectUpdated(const IObjectPtr& object)
	{
		if (StoryStatus::getStoryMode() == STORY_ARM_MOVING || StoryStatus::getStoryMode() == STORY_ARM_STOPPED) {
			objectWasUpdated = true;
		}

		if (object->getId() == TABLE_ID)
		{
			tableUpdated(object);
		} else {
			map<int,Box*>::iterator box = boxes.find(object->getId());
			if(box != boxes.end())
				box->second->updateModelObject(object);
		}
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
		if (StoryStatus::getProperty(ADDING_RIVER))
		{
			river->touchEvent(touchPoint);
		}
		else if (StoryStatus::getProperty(ADDING_STREET))
		{
			if (firstTouchDone && firstTableTouch.distance(touchPoint.getTouchPoint())> 0.05)
			{
				if (StoryStatus::getProperty(ADDING_STREET))
				{
					Road road = Road(firstTableTouch, touchPoint.getTouchPoint(), object->getPolygons().at(0)->getMathModel());
					btnManager->addButton(road.button);
					roads.push_back(road);
				}
				firstTouchDone = false;
				StoryStatus::setProperty(ADDING_STREET,false);
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
				if (StoryStatus::getProperty(WANT_TO_BURN))
				{
					touchedIdx->second->burn();
					StoryStatus::setProperty(WANT_TO_BURN, false);
					StoryStatus::setProperty(BURNING, true);
					
					StoryStatus::setStoryMode(STORY_MOVE_MODE);
					touchPoints.clear();
				}
				else
				{
					selectedBoxIdx = touchedIdx;
					ofVec3f spotCenter = floor->getPolygons()[0]->getMathModel().getPlane().project(object->getCenter());
					spotCenter.y -= 0.001f;
				
					vector<ofVec3f> vexs1 = object->getPolygon(kPolygonNameSideA)->getMathModel().getVertexs();
					vector<ofVec3f> vexs2 = object->getPolygon(kPolygonNameSideB)->getMathModel().getVertexs();
					float size = max(abs((vexs1[1] - vexs1[2]).length()),abs((vexs2[1] - vexs2[2]).length()))  + 0.3;
					spot.setSize(size);
					spot.setPosition(spotCenter);
				}
			}
			else if(selectedBoxIdx != touchedIdx)
			{
				touchedIdx->second->objectEvent(touchPoint,selectedBoxIdx->second->getBuildType());
				selectedBoxIdx = boxes.end(); 
				spot.setActive(false);
			}
		}
	}

	void Story::buttonPressed(const IButtonPtr& btn, const DataTouch& touchPoint)
	{
		menu.buttonEvent(btn,touchPoint,false);
		for(map<int,Box*>::iterator it = boxes.begin(); it != boxes.end(); ++it)
			it->second->buttonEvent(btn, false);
	}

	void Story::buttonReleased(const IButtonPtr& btn, const DataTouch& touchPoint)
	{
		menu.buttonEvent(btn,touchPoint,true);
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
	
	void Story::armStoppedMoving()
	{
		cout << "estado stopped" << endl;
		StoryStatus::setStoryMode(STORY_ARM_STOPPED);	// Setear por un rato el objectTracking para que ajuste a los objetos
		startTime = ofGetSystemTime();
	}

	void Story::setCamera(int camera)
	{
		if (camera > StoryConstants::CANT_CAMERAS) 
		{
			cout << "No existe la cámara número: " << camera << endl;
			return;
		} else if (StoryStatus::getStoryMode() == STORY_ARM_MOVING || StoryStatus::getStoryMode() == STORY_ARM_STOPPED) {
			cout << "Brazo ya se está moviendo. No ejecutar setCamera" << endl;
			return;			
		}

		previousMode = StoryStatus::getStoryMode();
		StoryStatus::setStoryMode(STORY_ARM_MOVING);

		Camera cam = StoryConstants::CAMERAS.at(camera - 1);
		this->armController->setArmPositionAndLookAt(cam.position,cam.lookAt);	
	}

	void Story::tableUpdated(const IObjectPtr& object) 
	{
		floor = object;
		Polygon3D table = object->getPolygons().at(0)->getMathModel();
		ofVec3f tableNormal = table.getPlane().getNormal();
		ofVec3f translateCanvas = tableNormal * -0.009;
		vector<ofVec3f> oldVexs = table.getVertexs();
		vector<ofVec3f> newVexs;
		for(int i = 0; i < 4; i++)
			newVexs.push_back(oldVexs.at(3 - i)+translateCanvas);

		table.setVertexs(newVexs);
		if (river == NULL)
			river = new Canvas(object->getId(),table, 512,512,ofColor(220,110,50),ofColor(75,140,250),20);
		else
			river->update(table);

	}


}
