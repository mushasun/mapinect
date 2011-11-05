#include "testApp.h"

#include "utils.h"

using namespace mapinect;

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
	vm.setup();
	arduino.setup();

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

//--------------------------------------------------------------
void testApp::fensterDraw()
{
	//lpmt.draw();
	vm.draw();
}

//--------------------------------------------------------------
void testApp::fensterUpdate()
{
	//lpmt.update();
	vm.update();
}

//--------------------------------------------------------------
void testApp::fensterKeyPressed(int key)
{
	//lpmt.keyPressed(key);
	vm.keyPressed(key);
}

//--------------------------------------------------------------
void testApp::fensterMouseMoved(int x, int y)
{
	//lpmt.mouseMoved(x, y);
	vm.mouseMoved(x, y);
}

//--------------------------------------------------------------
void testApp::fensterMouseDragged(int x, int y, int button)
{
	//lpmt.mouseDragged(x, y, button);
	vm.mouseDragged(x, y, button);
}

//--------------------------------------------------------------
void testApp::fensterMousePressed(int x, int y, int button)
{
	//lpmt.mousePressed(x, y, button);
	vm.mousePressed(x, y, button);
}

//--------------------------------------------------------------
void testApp::fensterMouseReleased(int x, int y, int button)
{
	//lpmt.mouseReleased(x, y, button);
	vm.mouseReleased(x, y, button);
}

//--------------------------------------------------------------
void testApp::fensterWindowResized(int w, int h)
{
	//lpmt.windowResized(w, h);
	vm.windowResized(w, h);
}

