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

	ard = new Arduino();

	valorArduino1 = 128;
	ard->sendMotor((char) valorArduino1, 1);
	valorArduino2 = 128;
	ard->sendMotor((char) valorArduino2, 2);
	valorArduino4 = 128;
	ard->sendMotor((char) valorArduino4, 4);
	
}

//--------------------------------------------------------------
void testApp::update() {
	gKinect->update();
	bool isKinectFrameNew = gKinect->isFrameNew();
	cv.update(isKinectFrameNew);
	//pcm.update(isKinectFrameNew);
}

//--------------------------------------------------------------
void testApp::draw()
{
	cv.draw();
	pcm.draw();
}

//--------------------------------------------------------------
void testApp::exit() {
	gKinect->close();
	delete ard;
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	char* x;
	char b = 127;
	int varAngle = 2;
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
	case 't':
		valorArduino2+=varAngle;
		cout << valorArduino2 << endl;
		//if(valorArduino2%10==0){
			ard->sendMotor((char) valorArduino2, 2);
		//}
		break;
	case 'g':
		valorArduino2-=varAngle;
		cout << valorArduino2 << endl;
		//if(valorArduino2%10==0){
			ard->sendMotor((char) valorArduino2, 2);
		//}
		break;
	case 'y':
		valorArduino4+=varAngle;
		cout << valorArduino4 << endl;
		//if(valorArduino4%10==0){
			ard->sendMotor((char) valorArduino4, 4);
		//}
		break;
	case 'h':
		valorArduino4-=varAngle;
		cout << valorArduino4 << endl;
		//if(valorArduino4%10==0){
			ard->sendMotor((char) valorArduino4, 4);
		//}
		break;
	case 'u':
		valorArduino1-=varAngle;
		cout << valorArduino1 << endl;
		//if(valorArduino2%10==0){
			ard->sendMotor((char) valorArduino1, 1);
		//}
		break;
	case 'j':
		valorArduino1+=varAngle;
		cout << valorArduino1 << endl;
		//if(valorArduino4%10==0){
			ard->sendMotor((char) valorArduino1, 1);
		//}
		break;
	case 'i':
		valorArduino8-=varAngle;
		cout << valorArduino8 << endl;
		//if(valorArduino2%10==0){
			ard->sendMotor((char) valorArduino8, 8);
		//}
		break;
	case 'k':
		valorArduino8+=varAngle;
		cout << valorArduino1 << endl;
		//if(valorArduino4%10==0){
			ard->sendMotor((char) valorArduino8, 8);
		//}
		break;
	case 'p':
		valorArduino1 = 128;
		ard->sendMotor((char) valorArduino1, 1);
		valorArduino2 = 128;
		ard->sendMotor((char) valorArduino2, 2);
		valorArduino4 = 128;
		ard->sendMotor((char) valorArduino4, 4);
	case 'm':
		ard->read();
	}
		
	//cv.keyPressed(key);
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

