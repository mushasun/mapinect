#include "mapinectApp.h"

#include "ofxFensterManager.h"

#include "ArmController.h"
#include "EventManager.h"
#include "Feature.h"
#include "Globals.h"
#include "log.h"
#include "Model.h"
#include "ofxKinect.h"
#include "pointUtils.h"
#include "ButtonManager.h"
#include "ModeManager.h"

namespace mapinect {

	//--------------------------------------------------------------
	mapinectApp::mapinectApp(ofxFenster* window, IApplication* app, VM* vm)
		: window(window), app(app), vm(vm)
	{
		LoadFeatures();
	}

	//--------------------------------------------------------------
	mapinectApp::~mapinectApp() {
		
	}

	//--------------------------------------------------------------
	void mapinectApp::setup() {
		gTransformation = new mapinect::Transformation();

		gKinect = new ofxKinect();
		if (IsFeatureKinectActive())
		{
			gKinect->init();
			gKinect->setVerbose(true);
			gKinect->open();
			// set Kinect looking down on startup
//No se debe cambiar el ángulo de tilt, debe quedar fijo
/*			angle = -28;
			gKinect->setCameraTiltAngle(angle);	*/
		}

		gModel = new mapinect::Model();

		ButtonManager* btnManager = new ButtonManager();
		app->btnManager = btnManager;

		ofSetWindowTitle("mapinect");
		cv.setup();
		pcm.setup(btnManager);
		arduino.setup();

		app->armController = new ArmController(&arduino);
		app->modeManager = new ModeManager(&pcm);
	}

	//--------------------------------------------------------------
	void mapinectApp::exit() {
		cv.exit();
		pcm.exit();
		arduino.exit();
		app->exit();

		if (IsFeatureKinectActive())
		{
			gKinect->close();
		}
	}

	//--------------------------------------------------------------
	void mapinectApp::update() {
		bool isKinectFrameNew = false;

		if (IsFeatureKinectActive())
		{
			gKinect->update();
			isKinectFrameNew = gKinect->isFrameNew();
		}

		cv.update(isKinectFrameNew);
		pcm.update(isKinectFrameNew);
		arduino.update();

	}

	//--------------------------------------------------------------
	void mapinectApp::draw()
	{
		window->setBackgroundColor(128, 128, 128);

		ofPushMatrix();
		ofTranslate(20.0f, 20.0f, 0.0f);
			cv.draw();
			pcm.draw();
		
			app->debugDraw();
		
			ofSetHexColor(0);
			stringstream reportStream;
			reportStream
				<< "           fps: " << ofGetFrameRate() << endl
				<< "    pcm thread: " << getPCMThreadStatus() << endl
				<< "objects thread: " << getObjectsThreadStatus() << endl;
			ofDrawBitmapString(reportStream.str(), 20, 520);
		ofPopMatrix();
	}

	//--------------------------------------------------------------
	void mapinectApp::keyPressed (int key) {

		cv.keyPressed(key);
		pcm.keyPressed(key);
		arduino.keyPressed(key);
	}

	//--------------------------------------------------------------
	void mapinectApp::keyReleased(int key)
	{
	}

	//--------------------------------------------------------------
	void mapinectApp::windowMoved(int x, int y)
	{
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
	void mapinectApp::mouseReleased(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void mapinectApp::dragEvent(ofDragInfo info)
	{
	}



	//--------------------------------------------------------------
	userApp::userApp(ofxFenster* window, IApplication* app)
		: window(window), app(app)
	{
		vm = new VM();
	}

	//--------------------------------------------------------------
	void userApp::exit()
	{
		app->exit();
	}

	//--------------------------------------------------------------
	void userApp::setup()
	{
		ofxFenster* win = ofxFensterManager::get()->createFenster(0, 0, 680, 600, OF_WINDOW);
		mapinectAppPtr = new mapinectApp(win, app, vm);
		win->addListener(mapinectAppPtr);
		mapinectAppPtr->setup();

		vm->setup();

		if (IsFeatureKinectActive())
		{
			double fxD, fyD, fxRGB, fyRGB; 
			float  cxD, cyD, cxRGB, cyRGB;
			ofVec3f TRGB;
			ofMatrix4x4 RRGB;
			getKinectCalibData(
				vm->getKinectCalibFile(),
				fxD, fyD, cxD, cyD,
				fxRGB, fyRGB, cxRGB, cyRGB,
				TRGB, RRGB);
			gKinect->getCalibration().setCalibValues(
				fxD, fyD, cxD, cyD,
				fxRGB, fyRGB, cxRGB, cyRGB,
				TRGB, RRGB);
		}

		// Use normalized textures
		ofEnableNormalizedTexCoords();

		//Habilita usar texturas transparentes
		glAlphaFunc(GL_GREATER, 0.02);
		glEnable(GL_ALPHA_TEST);

		app->setup();
		EventManager::suscribe(app);
		timer.start();
	}


	//--------------------------------------------------------------
	void userApp::draw()
	{
		window->setBackgroundColor(0, 0, 0);
		ofSetColor(255);
		
		// No se proyecta cuando el feature ENABLE_MAPPING_WHILE_MOVING está desactivado y la transformación no está estable (brazo en movimiento)
		if ( IsFeatureEnableMappingWhileMovingActive() || 
			 ( (!IsFeatureEnableMappingWhileMovingActive()) && gTransformation->getIsWorldTransformationStable() ) ) 
		{
			// Dibujar mesa y objetos detectados
			vm->setupView();
			app->draw();								
			vm->draw();
			((ButtonManager*)app->btnManager)->draw();	
			vm->endView();
		}

	}

	//--------------------------------------------------------------
	void userApp::update()
	{
		EventManager::fireEvents();

		vm->update();

		float elapsedTime = timer.stopResumeAndGetElapsedSeconds();
		app->update(elapsedTime);
	}

	//--------------------------------------------------------------
	void userApp::keyPressed(int key)
	{
		vm->keyPressed(key);
		app->keyPressed(key);

		switch (key)
		{
			/*********************
			  TOGGLE FULLSCREEN	 - F11
			*********************/
		case OF_KEY_F11:
				window->setFullscreen(window->getWindowMode() != OF_FULLSCREEN);
				break;
		}
	}

	//--------------------------------------------------------------
	void userApp::keyReleased(int key)
	{
		vm->keyReleased(key);
		app->keyReleased(key);
	}

	//--------------------------------------------------------------
	void userApp::windowMoved(int x, int y)
	{
		vm->windowMoved(x, y);
		app->windowMoved(x, y);
	}

	//--------------------------------------------------------------
	void userApp::mouseMoved(int x, int y)
	{
		vm->mouseMoved(x, y);
		app->mouseMoved(x, y);
	}

	//--------------------------------------------------------------
	void userApp::mouseDragged(int x, int y, int button)
	{
		vm->mouseDragged(x, y, button);
		app->mouseDragged(x, y, button);
	}

	//--------------------------------------------------------------
	void userApp::mousePressed(int x, int y, int button)
	{
		vm->mousePressed(x, y, button);
		app->mousePressed(x, y, button);
	}

	//--------------------------------------------------------------
	void userApp::mouseReleased(int x, int y, int button)
	{
		vm->mouseReleased(x, y, button);
		app->mouseReleased(x, y, button);
	}

	//--------------------------------------------------------------
	void userApp::dragEvent(ofDragInfo info)
	{
		vm->dragEvent(info);
		app->dragEvent(info);
	}



}
