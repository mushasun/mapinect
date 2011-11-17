#include "testApp.h"

#include "utils.h"
#include "TxManager.h"

//#define BOUNCING_BALL	1
#define BUILDINGS		1

#if BOUNCING_BALL
#include "BouncingBall.h"
#elif BUILDINGS
#include "Buildings.h"
#endif

using namespace mapinect;

//--------------------------------------------------------------
void testApp::setup() {
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

#if BOUNCING_BALL
	app = new bouncing::BouncingBall();
#elif BUILDINGS
	app = new buildings::Buildings();
#endif
	app->txManager = new TxManager(this->fenster);
	app->setup();

}

//--------------------------------------------------------------
void testApp::update() {
	gKinect->update();
	bool isKinectFrameNew = gKinect->isFrameNew();
	cv.update(isKinectFrameNew);
	pcm.update(isKinectFrameNew);
	arduino.update();
}

//--------------------------------------------------------------
void testApp::draw()
{
	cv.draw();
	pcm.draw();
}

//--------------------------------------------------------------
void testApp::exit() {
	cv.exit();
	pcm.exit();
	arduino.exit();
	app->exit();
	gKinect->close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
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
void testApp::mouseMoved(int x, int y)
{
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{
}

//--------------------------------------------------------------
void testApp::fensterSetup()
{
	vm.setup();
}


//--------------------------------------------------------------
void testApp::fensterDraw()
{
	vm.setupView();
	vm.draw();
	app->draw();
	vm.endView();
}

//--------------------------------------------------------------
void testApp::fensterUpdate()
{
	vm.update();
	app->update();
}

//--------------------------------------------------------------
void testApp::fensterKeyPressed(int key)
{
	vm.keyPressed(key);
	app->keyPressed(key);
}

//--------------------------------------------------------------
void testApp::fensterMouseMoved(int x, int y)
{
	app->mouseMoved(x, y);
}

//--------------------------------------------------------------
void testApp::fensterMouseDragged(int x, int y, int button)
{
	app->mouseDragged(x, y, button);
}

//--------------------------------------------------------------
void testApp::fensterMousePressed(int x, int y, int button)
{
	app->mousePressed(x, y, button);
}

//--------------------------------------------------------------
void testApp::fensterMouseReleased(int x, int y, int button)
{
	app->mouseReleased(x, y, button);
}

//--------------------------------------------------------------
void testApp::fensterWindowResized(int w, int h)
{
	vm.windowResized(w, h);
	app->windowResized(w, h);
}

