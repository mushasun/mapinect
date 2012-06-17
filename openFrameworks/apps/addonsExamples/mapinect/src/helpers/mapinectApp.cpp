#include "mapinectApp.h"

#include "ofxFensterManager.h"

#include "ArmController.h"
#include "EventManager.h"
#include "Feature.h"
#include "Globals.h"
#include "log.h"
#include "Model.h"
#include "ofxKinect.h"
#include "TxManager.h"
#include "pointUtils.h"

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
		gKinect = new ofxKinect();
		if (IsFeatureKinectActive())
		{
			gKinect->init();
			gKinect->setVerbose(true);
			gKinect->open();
			// set Kinect looking down on startup
			angle = -22;
			//gKinect->setCameraTiltAngle(angle);
		}


		gModel = new mapinect::Model();
		ofSetWindowTitle("mapinect");
		cv.setup();
		pcm.setup();
		arduino.setup();

		app->txManager = new TxManager();
		app->armController = new ArmController(&arduino);

		if (IsFeatureMoveArmActive()) {
			// Set transformation matrix to apply to point cloud, in pointUtils::getPartialCloudRealCoords
			setTransformMatrix(arduino.getWorldTransformation());	
			// Set transformation matrix in VM to apply to Modelview matrix
			vm->setInverseWorldTransformationMatrix(arduino.getWorldTransformation());
		}

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

		if  (IsFeatureMoveArmActive()) {
			// Set transformation matrix to apply to point cloud, in pointUtils::getPartialCloudRealCoords
			setTransformMatrix(arduino.getWorldTransformation());	// Method from pointUtils	
			// Set transformation matrix in VM to apply to Modelview matrix
			vm->setInverseWorldTransformationMatrix(arduino.getWorldTransformation());
		}

	}

	//--------------------------------------------------------------
	void mapinectApp::draw()
	{
		window->setBackgroundColor(128, 128, 128);

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
			if (IsFeatureKinectActive())
			{
				gKinect->setCameraTiltAngle(angle);
				printf("Current Kinect tilt angle: %d\n", angle);
			}
		}
		
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
	PCM* mapinectApp::getPCM()
	{
		return &pcm;
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
	void getKinectCalibData(const string& kinect_calib_file,
							double& d_fx, double& d_fy, float& d_cx, float& d_cy,
							double& rgb_fx, double& rgb_fy, float& rgb_cx, float& rgb_cy,
							ofVec3f& T, ofMatrix4x4& R);
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
			double fx_d, fy_d, fx_rgb, fy_rgb; 
			float  cx_d, cy_d, cx_rgb, cy_rgb;
			ofVec3f T_rgb;
			ofMatrix4x4 R_rgb;
			getKinectCalibData(vm->getKinectCalibFile(), fx_d, fy_d, cx_d, cy_d,
										fx_rgb, fy_rgb, cx_rgb, cy_rgb, T_rgb, R_rgb);
			gKinect->getCalibration().setCalibValues( fx_d, fy_d, cx_d, cy_d,
										fx_rgb, fy_rgb, cx_rgb, cy_rgb, T_rgb, R_rgb);
		}

		app->setup();
	}


	//--------------------------------------------------------------
	void userApp::draw()
	{
		window->setBackgroundColor(0, 0, 0);
		ofSetColor(255);
		vm->setupView();
		vm->draw();
		app->txManager->updateVideoTextures();
		app->draw();
		vm->endView();
	}

	//--------------------------------------------------------------
	void userApp::update()
	{
		EventManager::fireEvents(app, mapinectAppPtr->getPCM());

		vm->update();
		app->update();
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
		case 305:
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



	//--------------------------------------------------------------
	// Get calibration parameters for Kinect's rgb and depth cameras
	//		Camara Lucida, www.camara-lucida.com.ar
	void getKinectCalibData(const string& kinect_calib_file,
							double& d_fx, double& d_fy, float& d_cx, float& d_cy,
							double& rgb_fx, double& rgb_fy, float& rgb_cx, float& rgb_cy,
							ofVec3f& T, ofMatrix4x4& R)
	{
		// Load matrices from Kinect's calibration file		
		CvMat* rgb_intrinsics = (CvMat*) cvLoad(kinect_calib_file.c_str(), NULL, "rgb_intrinsics");
		// CvMat* rgb_size = (CvMat*) cvLoad(kinect_calib_file, NULL_, "rgb_size");		// Size is 640x480 fixed for both RGB and IR
		CvMat* depth_intrinsics = (CvMat*) cvLoad(kinect_calib_file.c_str(), NULL, "depth_intrinsics");
		//CvMat* depth_size = (CvMat*) cvLoad(kinect_calib_file, NULL, "depth_size");	// Size is 640x480 fixed for both RGB and IR
		CvMat* kinect_R = (CvMat*) cvLoad(kinect_calib_file.c_str(), NULL, "R");
		CvMat* kinect_T = (CvMat*) cvLoad(kinect_calib_file.c_str(), NULL, "T");
			
		// Obtain RGB intrinsic parameters
		rgb_fx = (double) cvGetReal2D(rgb_intrinsics, 0, 0);
		rgb_fy = (double) cvGetReal2D(rgb_intrinsics, 1, 1);
		rgb_cx = (float) cvGetReal2D(rgb_intrinsics, 0, 2);
		rgb_cy = (float) cvGetReal2D(rgb_intrinsics, 1, 2);

		// Obtain Depth intrinsic parameters
		d_fx = (double) cvGetReal2D(depth_intrinsics, 0, 0);
		d_fy = (double) cvGetReal2D(depth_intrinsics, 1, 1);
		d_cx = (float) cvGetReal2D(depth_intrinsics, 0, 2);
		d_cy = (float) cvGetReal2D(depth_intrinsics, 1, 2);


		T.x = (float) cvGetReal2D(kinect_T,0,0);
		T.y = (float) cvGetReal2D(kinect_T,1,0);
		T.z = (float) cvGetReal2D(kinect_T,2,0);

		R.set((float)cvGetReal2D(kinect_R, 0, 0), (float)cvGetReal2D(kinect_R, 0, 1), (float)cvGetReal2D(kinect_R, 0, 2), 0,
			  (float)cvGetReal2D(kinect_R, 1, 0), (float)cvGetReal2D(kinect_R, 1, 1), (float)cvGetReal2D(kinect_R, 1, 2), 0,
			  (float)cvGetReal2D(kinect_R, 2, 0), (float)cvGetReal2D(kinect_R, 2, 1), (float)cvGetReal2D(kinect_R, 2, 2), 0,
			  0, 0, 0, 1);

		// Release matrices loaded with Kinect's calib values 
		cvReleaseMat(&rgb_intrinsics);
		cvReleaseMat(&depth_intrinsics);
		cvReleaseMat(&kinect_R);
		cvReleaseMat(&kinect_T); 
	}

}
