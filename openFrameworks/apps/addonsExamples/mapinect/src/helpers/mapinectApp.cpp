#include "mapinectApp.h"

#include "ArmController.h"
#include "EventManager.h"
#include "Feature.h"
#include "Globals.h"
#include "log.h"
#include "Model.h"
#include "ofxKinect.h"
#include "TxManager.h"

namespace mapinect {

	//--------------------------------------------------------------
	mapinectApp::mapinectApp(IApplication* app) {
		this->app = app;

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

		app->txManager = new TxManager(fenster);
		app->armController = new ArmController(&arduino);
		app->setup();

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

		EventManager::fireEvents(app);
		
		app->update();
	}

	//--------------------------------------------------------------
	void mapinectApp::draw()
	{
		cv.draw();
		pcm.draw();
		
		
		app->debugDraw();
		
		//ofEnableAlphaBlending();  
		//ofSetColor(255,255,255,128);
		//gKinect->draw(0,0);
		//ofDisableAlphaBlending();

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
				//gKinect->setCameraTiltAngle(angle);
				//printf("Current Kinect tilt angle: %d\n", angle);
			}
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

		if (IsFeatureKinectActive())
		{
			double fx_d, fy_d, fx_rgb, fy_rgb; 
			float  cx_d, cy_d, cx_rgb, cy_rgb;
			ofVec3f T_rgb;
			ofMatrix4x4 R_rgb;
			getKinectCalibData(const_cast<char*>(VM::kinect_calib_file.c_str()), fx_d, fy_d, cx_d, cy_d,
										fx_rgb, fy_rgb, cx_rgb, cy_rgb, T_rgb, R_rgb);
			gKinect->getCalibration().setCalibValues( fx_d, fy_d, cx_d, cy_d,
										fx_rgb, fy_rgb, cx_rgb, cy_rgb, T_rgb, R_rgb);
		}
	}


	//--------------------------------------------------------------
	void mapinectApp::fensterDraw()
	{
		vm.setupView();
		vm.draw();
		app->txManager->updateVideoTextures();
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

	// Get calibration parameters for Kinect's rgb and depth cameras
	//		Camara Lucida, www.camara-lucida.com.ar
	void mapinectApp::getKinectCalibData(char* kinect_calib_file, double& d_fx, double& d_fy, float& d_cx, float& d_cy,
									double& rgb_fx, double& rgb_fy, float& rgb_cx, float& rgb_cy,
									ofVec3f& T, ofMatrix4x4& R){
		// Load matrices from Kinect's calibration file		
		CvMat* rgb_intrinsics = (CvMat*) cvLoad(kinect_calib_file, NULL, "rgb_intrinsics");
		// CvMat* rgb_size = (CvMat*) cvLoad(kinect_calib_file, NULL_, "rgb_size");		// Size is 640x480 fixed for both RGB and IR
		CvMat* depth_intrinsics = (CvMat*) cvLoad(kinect_calib_file, NULL, "depth_intrinsics");
		//CvMat* depth_size = (CvMat*) cvLoad(kinect_calib_file, NULL, "depth_size");	// Size is 640x480 fixed for both RGB and IR
		CvMat* kinect_R = (CvMat*) cvLoad(kinect_calib_file, NULL, "R");
		CvMat* kinect_T = (CvMat*) cvLoad(kinect_calib_file, NULL, "T");
			
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
