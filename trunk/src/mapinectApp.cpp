#include "mapinectApp.h"

#include "utils.h"
#include "TxManager.h"

namespace mapinect {

	//--------------------------------------------------------------
	mapinectApp::mapinectApp(IApplication* app) {
		this->app = app;
	}

	//--------------------------------------------------------------
	mapinectApp::~mapinectApp() {
		
	}

	//--------------------------------------------------------------
	void mapinectApp::setup() {
		gKinect = new ofxKinect();
		gKinect->init();
		gKinect->setVerbose(true);
		gKinect->open();

		// zero the tilt on startup
		angle = 0;

		gModel = new mapinect::Model();
		ofSetWindowTitle("mapinect");
		cv.setup();
		pcm.setup();
		arduino.setup();

		app->txManager = new TxManager(this->fenster);
		app->setup();

	}

	//--------------------------------------------------------------
	void mapinectApp::exit() {
		cv.exit();
		pcm.exit();
		arduino.exit();
		app->exit();
		gKinect->close();
	}

	//--------------------------------------------------------------
	void mapinectApp::update() {
		gKinect->update();
		bool isKinectFrameNew = gKinect->isFrameNew();
		cv.update(isKinectFrameNew);
		pcm.update(isKinectFrameNew);
		arduino.update();
	}

	//--------------------------------------------------------------
	void mapinectApp::draw()
	{
		cv.draw();
		pcm.draw();
		//app->debugDraw();
	}

	//--------------------------------------------------------------
	void mapinectApp::keyPressed (int key) {
		int angleVariation = 0;
		switch (key) {
		case OF_KEY_UP:
			angleVariation++;
			break;
		case OF_KEY_DOWN:
			angleVariation--;
			break;
		}
		if (angleVariation != 0) {
			angle = ofClamp(angle + angleVariation, -30, 30);
			gKinect->setCameraTiltAngle(angle);
		}
		
		cv.keyPressed(key);
		pcm.keyPressed(key);
		arduino.keyPressed(key);
	}

	//--------------------------------------------------------------
	void mapinectApp::mouseMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void mapinectApp::mouseDragged(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void mapinectApp::mousePressed(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void mapinectApp::windowResized(int w, int h)
	{
	}

	//--------------------------------------------------------------
	void mapinectApp::mouseReleased(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void mapinectApp::fensterSetup()
	{
		vm.setup();
	}


	//--------------------------------------------------------------
	void mapinectApp::fensterDraw()
	{
		vm.setupView();
		vm.draw();
		app->draw();
		vm.endView();
	}

	//--------------------------------------------------------------
	void mapinectApp::fensterUpdate()
	{
		vm.update();
		app->update();
	}

	//--------------------------------------------------------------
	void mapinectApp::fensterKeyPressed(int key)
	{
		vm.keyPressed(key);
		app->keyPressed(key);
	}

	//--------------------------------------------------------------
	void mapinectApp::fensterMouseMoved(int x, int y)
	{
		app->mouseMoved(x, y);
	}

	//--------------------------------------------------------------
	void mapinectApp::fensterMouseDragged(int x, int y, int button)
	{
		app->mouseDragged(x, y, button);
	}

	//--------------------------------------------------------------
	void mapinectApp::fensterMousePressed(int x, int y, int button)
	{
		app->mousePressed(x, y, button);
	}

	//--------------------------------------------------------------
	void mapinectApp::fensterMouseReleased(int x, int y, int button)
	{
		app->mouseReleased(x, y, button);
	}

	//--------------------------------------------------------------
	void mapinectApp::fensterWindowResized(int w, int h)
	{
		vm.windowResized(w, h);
		app->windowResized(w, h);
	}

}
