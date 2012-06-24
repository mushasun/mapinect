#include "VisualizerApp.h"

#include "Globals.h"
#include "pointUtils.h"		// Esto vamos a tener que sacarlo

namespace visualizer {
	
	//--------------------------------------------------------------
	VisualizerApp::VisualizerApp() {
		floor = NULL;
	}

	//--------------------------------------------------------------
	VisualizerApp::~VisualizerApp() {
		if (floor != NULL) {
			delete floor;
		}
		for (map<int, Box*>::iterator i = boxes.begin(); i != boxes.end(); i++) {
			delete i->second;
		}
		boxes.clear();
	}

	//--------------------------------------------------------------
	void VisualizerApp::setup() {
		fftSmoothed = new float[8192];
		for (int i = 0; i < 8192; i++){
			fftSmoothed[i] = 0;
		}	
		nBandsToGet = 128;

		music.loadSound("sounds/miles.mp3");
		music.setVolume(1.0f);
		music.setLoop(true);
		music.play();

		   // Global openFrameworks settings
		ofEnableAlphaBlending();
		ofEnableSmoothing();
		ofLogLevel(OF_LOG_NOTICE);    
		ofSetFrameRate(120.0f);

		vector<ofColor> colors;
		for(int i = 0; i < 5; i ++)
			colors.push_back(ofColor(rand()%255,rand()%255,rand()%255));
		vis.setup(colors);
	}

	//--------------------------------------------------------------
	void VisualizerApp::exit() {
	}

	//--------------------------------------------------------------
	void VisualizerApp::debugDraw()
	{
		/*ofClear(0,0,0);
		vis.draw(ofVec3f(0,0,0), ofVec3f(600,0,0),ofVec3f(600,600,0),ofVec3f(0,600,0));
		return;*/
		// Esto vamos a tener que sacarlo
		map<int, DataTouch> keep;
		for (map<int, DataTouch>::const_iterator it = touchPoints.begin(); it != touchPoints.end(); ++it)
		{
			if (it->second.getType() == kTouchTypeStarted)
				ofSetHexColor(0xFF0000);
			else if (it->second.getType() == kTouchTypeHolding)
				ofSetHexColor(0x00FF00);
			else
				ofSetHexColor(0x0000FF);
			ofVec2f s = getScreenCoords(it->second.getTouchPoint());
			ofCircle(s.x, s.y, 4);
			if (it->second.getType() != kTouchTypeReleased)
				keep.insert(make_pair(it->first, it->second));
		}
		touchPoints = keep;
	}

	//--------------------------------------------------------------
	void VisualizerApp::draw()
	{
		if (floor != NULL) {
			floor->draw();
		}

		for (map<int, Box*>::iterator iter = boxes.begin(); iter != boxes.end(); iter++) {
			(iter->second)->draw(*floor);
		}
	}


	//--------------------------------------------------------------
	void VisualizerApp::update() {
		vis.update();
		ofSoundUpdate();
		int chanel[3] = { 1, 7, 27}; //bass - snare - symbol

		int boxesSize = boxes.size();
		if(boxesSize > 0)
		{
			int j = 0;
			int slotSize = nBandsToGet / boxesSize;

			float* val = ofSoundGetSpectrum(nBandsToGet);	
			for (int i = 0;i < nBandsToGet; i++){	
				j = i / slotSize;

				fftSmoothed[i] *= 0.96f;
				if (fftSmoothed[i] < val[i]) 
					fftSmoothed[i] = val[i];	
			}

			j = 0;
			for (map<int, Box*>::iterator iter = boxes.begin(); iter != boxes.end(); iter++) {
				float prog = fftSmoothed[chanel[j%3]];
				if(prog > 1)
					prog = 1;
				(iter->second)->update(prog);
				j++;
			}
		}
	}

	//--------------------------------------------------------------
	void VisualizerApp::keyPressed(int key)
	{
	}

	//--------------------------------------------------------------
	void VisualizerApp::keyReleased(int key)
	{
	}

	//--------------------------------------------------------------
	void VisualizerApp::windowMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void VisualizerApp::mouseMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void VisualizerApp::mouseDragged(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void VisualizerApp::mousePressed(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void VisualizerApp::mouseReleased(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void VisualizerApp::dragEvent(ofDragInfo info)
	{
	}

	//--------------------------------------------------------------
	void VisualizerApp::objectDetected(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			if (floor != NULL)
			{
				delete floor;
				floor = NULL;
			}
			if (floor == NULL)
			{
				floor = new Floor(object->getPolygons()[0]);
			}
		}
		else
		{
			if (boxes.find(object->getId()) == boxes.end())
			{
				boxes[object->getId()] = new Box(object);
			}
		}
	}

	//--------------------------------------------------------------
	void VisualizerApp::objectUpdated(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			if (floor != NULL)
			{
				floor->updateModelObject(object->getPolygons()[0]);
			}
		}
		else
		{
			map<int, Box*>::iterator b = boxes.find(object->getId());
			if (b != boxes.end())
			{
				b->second->updateModelObject(object);
			}
		}
	}

	//--------------------------------------------------------------
	void VisualizerApp::objectLost(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			if (floor != NULL)
			{
				delete floor;
				floor = NULL;
			}
		}
		else
		{
			if (boxes.find(object->getId()) != boxes.end())
			{
				boxes.erase(object->getId());
			}
		}
	}

	//--------------------------------------------------------------
	void VisualizerApp::objectMoved(const IObjectPtr& object, const DataMovement& movement)
	{
		if (object->getId() == TABLE_ID)
		{
			if (floor != NULL)
			{
				floor->updateModelObject(object->getPolygons()[0]);
			}
		}
		else
		{
			map<int, Box*>::iterator b = boxes.find(object->getId());
			if (b != boxes.end())
			{
				b->second->updateModelObject(object);
			}
		}
	}
	
	//--------------------------------------------------------------
	void VisualizerApp::objectTouched(const IObjectPtr& object, const DataTouch& touchPoint)
	{
		map<int, DataTouch>::iterator it = touchPoints.find(touchPoint.getId());
		if (it == touchPoints.end())
		{
			assert(touchPoint.getType() == kTouchTypeStarted);
			touchPoints.insert(make_pair(touchPoint.getId(), touchPoint));
		}
		else
		{
			it->second = touchPoint;
		}
	}
}
