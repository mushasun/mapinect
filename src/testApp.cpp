#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup() {
	kinect.init();
	kinect.setVerbose(true);
	kinect.open();

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);

	cv.setup(&kinect);
	pcm.setup(&kinect);
	lpmt.setup(&kinect);
}

//--------------------------------------------------------------
void testApp::update() {
	kinect.update();
	bool isKinectFrameNew = kinect.isFrameNew();
	cv.update(isKinectFrameNew);
	pcm.update(isKinectFrameNew);
}

//--------------------------------------------------------------
void testApp::draw()
{
	cv.draw();
	pcm.draw();
}

//--------------------------------------------------------------
void testApp::exit() {
	//kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
	case 'w':
		kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
		break;
	case 'o':
		kinect.setCameraTiltAngle(angle);	// go back to prev tilt
		kinect.open();
		break;
	case OF_KEY_UP:
		angle++;
		if(angle>30) angle=30;
		kinect.setCameraTiltAngle(angle);
		break;
	case OF_KEY_DOWN:
		angle--;
		if(angle<-30) angle=-30;
		kinect.setCameraTiltAngle(angle);
		break;
	}
	cv.keyPressed(key);
	pcm.keyPressed(key);
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
	lpmt.draw();
}

//--------------------------------------------------------------
void testApp::fensterUpdate()
{
	lpmt.update();
}

//--------------------------------------------------------------
void testApp::fensterKeyPressed(int key)
{
	lpmt.keyPressed(key);
}

//--------------------------------------------------------------
void testApp::fensterMouseMoved(int x, int y)
{
	lpmt.mouseMoved(x, y);
}

//--------------------------------------------------------------
void testApp::fensterMouseDragged(int x, int y, int button)
{
	lpmt.mouseDragged(x, y, button);
}

//--------------------------------------------------------------
void testApp::fensterMousePressed(int x, int y, int button)
{
	lpmt.mousePressed(x, y, button);
}

//--------------------------------------------------------------
void testApp::fensterMouseReleased(int x, int y, int button)
{
	lpmt.mouseReleased(x, y, button);
}

//--------------------------------------------------------------
void testApp::fensterWindowResized(int w, int h)
{
	lpmt.windowResized(w, h);
}

