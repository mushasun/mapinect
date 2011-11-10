#include "testApp.h"

#include "utils.h"

using namespace mapinect;

bool setupTexture;

#define BOUNCING_BALL		1
#define BUILDINGS			2

static int APPLICATION = BOUNCING_BALL;

//--------------------------------------------------------------
void testApp::setup() {
	gKinect = new ofxKinect();
	gKinect->init();
	gKinect->setVerbose(true);
	gKinect->open();

	// zero the tilt on startup
	angle = 0;
	//gKinect->setCameraTiltAngle(angle);

	gModel = new mapinect::Model();
	ofSetWindowTitle("mapinect");
	cv.setup();
	pcm.setup();
//	lpmt.setup();
	if (APPLICATION == BOUNCING_BALL) {
		bb.setup();
	}
	else if (APPLICATION == BUILDINGS) {
		bu.setup(this->fenster);
	}
	
	arduino.setup();

	setupTexture = true;
}

//--------------------------------------------------------------
void testApp::update() {
	gKinect->update();
	bool isKinectFrameNew = gKinect->isFrameNew();
	cv.update(isKinectFrameNew);
	//pcm.update(isKinectFrameNew);
	arduino.update();
}

//--------------------------------------------------------------
void testApp::draw()
{
	cv.draw();
	pcm.draw();
	arduino.draw();

}

//--------------------------------------------------------------
void testApp::exit() {
	pcm.exit();
	gKinect->close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
	case 'w':
		gKinect->enableDepthNearValueWhite(!gKinect->isDepthNearValueWhite());
		break;
	case 'o':
		gKinect->setCameraTiltAngle(angle);	// go back to prev tilt
		gKinect->open();
		break;
	case OF_KEY_UP:
		angle++;
		if(angle>30) angle=30;
		gKinect->setCameraTiltAngle(angle);
		break;
	case OF_KEY_DOWN:
		angle--;
		if(angle<-30) angle=-30;
		gKinect->setCameraTiltAngle(angle);
		break;
	}
		
	//cv.keyPressed(key);
	pcm.keyPressed(key);
	arduino.keyPressed(key);
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y)
{
	cv.mouseMoved(x, y);
	pcm.mouseMoved(x, y);
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{
	cv.mouseDragged(x, y, button);
	pcm.mouseDragged(x, y, button);
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{
	cv.mousePressed(x, y, button);
	pcm.mousePressed(x, y, button);
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{
	cv.windowResized(w, h);
	pcm.windowResized(w, h);
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{
	cv.mouseReleased(x, y, button);
	pcm.mouseReleased(x, y, button);
}

//
void testApp::fensterSetup()
{
	//vm.setup(this->fenster);
}


//--------------------------------------------------------------
void testApp::fensterDraw()
{
	//lpmt.draw();
	if (APPLICATION == BOUNCING_BALL) {
		bb.draw();
	}
	else if (APPLICATION == BUILDINGS) {
		bu.draw();
	}

}

//--------------------------------------------------------------
void testApp::fensterUpdate()
{
	//lpmt.update();
	if (APPLICATION == BOUNCING_BALL) {
		bb.update();
	}
	else if (APPLICATION == BUILDINGS) {
		bu.update();
	}

/*	if (setupTexture) {
		// First load image texture
		GLuint textureID = vm.loadImageTexture("ofTheo.jpg");
		setupTexture = false;
	}
	*/
}

//--------------------------------------------------------------
void testApp::fensterKeyPressed(int key)
{
	//lpmt.keyPressed(key);
	if (APPLICATION == BOUNCING_BALL) {
		bb.keyPressed(key);
	}
	else if (APPLICATION == BUILDINGS) {
		bu.keyPressed(key);
	}
}

//--------------------------------------------------------------
void testApp::fensterMouseMoved(int x, int y)
{
	//lpmt.mouseMoved(x, y);
	if (APPLICATION == BOUNCING_BALL) {
		bb.mouseMoved(x, y);
	}
	else if (APPLICATION == BUILDINGS) {
		bu.mouseMoved(x, y);
	}
}

//--------------------------------------------------------------
void testApp::fensterMouseDragged(int x, int y, int button)
{
	//lpmt.mouseDragged(x, y, button);
	if (APPLICATION == BOUNCING_BALL) {
		bb.mouseDragged(x, y, button);
	}
	else if (APPLICATION == BUILDINGS) {
		bu.mouseDragged(x, y, button);
	}
}

//--------------------------------------------------------------
void testApp::fensterMousePressed(int x, int y, int button)
{
	//lpmt.mousePressed(x, y, button);
	if (APPLICATION == BOUNCING_BALL) {
		bb.mousePressed(x, y, button);
	}
	else if (APPLICATION == BUILDINGS) {
		bu.mousePressed(x, y, button);
	}
}

//--------------------------------------------------------------
void testApp::fensterMouseReleased(int x, int y, int button)
{
	//lpmt.mouseReleased(x, y, button);
	if (APPLICATION == BOUNCING_BALL) {
		bb.mouseReleased(x, y, button);
	}
	else if (APPLICATION == BUILDINGS) {
		bu.mouseReleased(x, y, button);
	}
}

//--------------------------------------------------------------
void testApp::fensterWindowResized(int w, int h)
{
	//lpmt.windowResized(w, h);
	if (APPLICATION == BOUNCING_BALL) {
		bb.windowResized(w, h);
	}
	else if (APPLICATION == BUILDINGS) {
		bu.windowResized(w, h);
	}
}

