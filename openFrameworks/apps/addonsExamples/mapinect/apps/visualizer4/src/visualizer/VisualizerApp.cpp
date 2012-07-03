#include "VisualizerApp.h"

#include "Globals.h"
#include "pointUtils.h"		// Esto vamos a tener que sacarlo
#include "SimpleButton.h"
#include "DraggableButton.h"

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

		/*ding.loadSound("sounds/ding.wav");
		ding.setVolume(1.0f);
		ding.setLoop(false);

		dong.loadSound("sounds/dong.wav");
		dong.setVolume(1.0f);
		dong.setLoop(false);*/

	//	sample1.loadSound("sounds/WAV/DrumMix_CK02_94bpm_03.wav");
	//	sample1.setVolume(1.0f);
	//	sample1.setLoop(true);
	//	//sample1.play();

	//	sample2.loadSound("sounds/WAV/Lead_CK02_94bpm_04.wav");
	//	sample2.setVolume(1.0f);
	//	sample2.setLoop(true);
	//	//sample2.play();

	//	sample3.loadSound("sounds/WAV/Strings_CK02_94bpm.wav");
	//	sample3.setVolume(1.0f);
	//	sample3.setLoop(true);
	////	sample3.play();

	//	sample4.loadSound("sounds/WAV/Timpani_CK02_94bpm.wav");
	//	sample4.setVolume(1.0f);
	//	sample4.setLoop(true);
//		sample4.play();
		//music.play();

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
		//if (floor != NULL) {
		//	floor->draw();
		//}

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

				Polygon3D pol(object->getPolygons()[0]->getMathModel());

				//Cuadrante 1
				vector<ofVec3f> cuadrant1;
				cuadrant1.push_back(pol.getVertexs()[0]);
				cuadrant1.push_back(pol.getVertexs()[1] + ((pol.getVertexs()[0] - pol.getVertexs()[1])/2));
				cuadrant1.push_back(object->getCenter());
				cuadrant1.push_back(pol.getVertexs()[3] + ((pol.getVertexs()[0] - pol.getVertexs()[3])/2));
				Polygon3D btn1(cuadrant1);

				SimpleButton sb1(btn1,
								ofColor(190,0,0),
								ofColor(255,0,0));

				//Cuadrante 2
				vector<ofVec3f> cuadrant2;
				cuadrant2.push_back(pol.getVertexs()[1] + ((pol.getVertexs()[0] - pol.getVertexs()[1])/2));
				cuadrant2.push_back(pol.getVertexs()[1]);
				
				cuadrant2.push_back(pol.getVertexs()[2] + ((pol.getVertexs()[1] - pol.getVertexs()[2])/2));
				cuadrant2.push_back(object->getCenter());
				Polygon3D btn2(cuadrant2);

				SimpleButton sb2(btn2,
								ofColor(0,190,0),
								ofColor(0,255,0));

				////Cuadrante 3
				vector<ofVec3f> cuadrant3;
				cuadrant3.push_back(object->getCenter());
				cuadrant3.push_back(pol.getVertexs()[2] + ((pol.getVertexs()[1] - pol.getVertexs()[2])/2));
				cuadrant3.push_back(pol.getVertexs()[2]);
				cuadrant3.push_back(pol.getVertexs()[3] + ((pol.getVertexs()[2] - pol.getVertexs()[3])/2));
				Polygon3D btn3(cuadrant3);

				SimpleButton sb3(btn3,
								ofColor(0,0,190),
								ofColor(0,0,255));

				////Cuadrante 4
				vector<ofVec3f> cuadrant4;
				cuadrant4.push_back(pol.getVertexs()[3] + ((pol.getVertexs()[0] - pol.getVertexs()[3])/2));
				cuadrant4.push_back(object->getCenter());
				cuadrant4.push_back(pol.getVertexs()[3] + ((pol.getVertexs()[2] - pol.getVertexs()[3])/2));
				cuadrant4.push_back(pol.getVertexs()[3]);
				Polygon3D btn4(cuadrant4);

				SimpleButton sb4(btn4,
								ofColor(190,190,0),
								ofColor(255,255,0));

				this->btnManager->addButton(SimpleButtonPtr(new SimpleButton(sb1)));
				this->btnManager->addButton(SimpleButtonPtr(new SimpleButton(sb2)));
				this->btnManager->addButton(SimpleButtonPtr(new SimpleButton(sb3)));
				this->btnManager->addButton(SimpleButtonPtr(new SimpleButton(sb4)));


				////DRAG BUTTON////////////////
				//Draggable 1
				vector<ofVec3f> draggable1;
				draggable1.push_back(pol.getVertexs()[0]);
				draggable1.push_back(pol.getVertexs()[1] + ((pol.getVertexs()[0] - pol.getVertexs()[1])/2));
				draggable1.push_back(object->getCenter());
				draggable1.push_back(pol.getVertexs()[3] + ((pol.getVertexs()[0] - pol.getVertexs()[3])/2));
				Polygon3D drag1(draggable1);

				DraggableButton d1(drag1,
								ofColor(128,0,0),
								ofColor(255,0,0));

				//this->btnManager->addButton(DraggableButtonPtr(new DraggableButton(d1)));
			}
		}
		else
		{
			
			if (boxes.find(object->getId()) == boxes.end())
			{
				boxes[object->getId()] = new Box(object);
			}
			if(!music.getIsPlaying())
			{
				music.play();
				//music.setVolume(object->getCenter().z);
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
				//music.setVolume(object->getCenter().z);
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
			//assert(touchPoint.getType() == kTouchTypeStarted);
			touchPoints.insert(make_pair(touchPoint.getId(), touchPoint));
		}
		else
		{
			it->second = touchPoint;
		}
	}

	void VisualizerApp::buttonPressed(const IButtonPtr& btn)
	{
		music.setSpeed(btn->getId());
		if(btn->getId() == 1 && !music.getIsPlaying())
			music.play();
		if(btn->getId() == 4)
			music.stop();

		int id = btn->getId();
		switch(id)
		{
			case 0: 
				if(sample1.getIsPlaying())
					sample1.stop();
				else
					sample1.play();
				break;
			case 1: 
				if(sample2.getIsPlaying())
					sample2.stop();
				else
					sample2.play();
				break;
			case 2: 
				if(sample3.getIsPlaying())
					sample3.stop();
				else
					sample3.play();
				break;
			case 3: 
				if(sample4.getIsPlaying())
					sample4.stop();
				else
					sample4.play();
				break;
		}
	}
	void VisualizerApp::buttonReleased(const IButtonPtr& btn)
	{
		//dong.play();
	}
}
