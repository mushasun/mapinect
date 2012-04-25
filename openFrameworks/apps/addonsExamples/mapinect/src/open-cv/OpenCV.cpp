#include "OpenCV.h"
#include "utils.h"
#include "ofGraphicsUtils.h"
namespace mapinect {

	//--------------------------------------------------------------
	void OpenCV::setup() {

		colorImg.allocate(gKinect->width, gKinect->height);
		grayImage.allocate(gKinect->width, gKinect->height);
		grayThresh.allocate(gKinect->width, gKinect->height);
		grayThreshFar.allocate(gKinect->width, gKinect->height);
		grayBg.allocate(gKinect->width, gKinect->height);
		grayDiff.allocate(gKinect->width, gKinect->height);

		nearThreshold = 233;
		farThreshold  = 208;
		bThreshWithOpenCV = true;

		threshold = 80;
	}

	//--------------------------------------------------------------
	void OpenCV::exit() {
		
	}

	//--------------------------------------------------------------
	void OpenCV::update(bool isKinectFrameNew) {

		//if(isKinectFrameNew)
		//{
		//	ofBackground(100, 100, 100);

		//	grayImage.setFromPixels(gKinect->getDepthPixels(), gKinect->width, gKinect->height);

		//	//we do two thresholds - one for the far plane and one for the near plane
		//	//we then do a cvAnd to get the pixels which are a union of the two thresholds.	
		//	if( bThreshWithOpenCV ) {
		//		grayThreshFar = grayImage;
		//		grayThresh = grayImage;
		//		grayThresh.threshold(nearThreshold, true);
		//		grayThreshFar.threshold(farThreshold);
		//		cvAnd(grayThresh.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		//	}
		//	else{

		//		//or we do it ourselves - show people how they can work with the pixels

		//		unsigned char * pix = grayImage.getPixels();
		//		int numPixels = grayImage.getWidth() * grayImage.getHeight();

		//		for(int i = 0; i < numPixels; i++){
		//			if( pix[i] < nearThreshold && pix[i] > farThreshold ){
		//				pix[i] = 255;
		//			}else{
		//				pix[i] = 0;
		//			}
		//		}
		//	}

		//	//update the cv image
		//	grayImage.flagImageChanged();

		//	// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		//	// also, find holes is set to true so we will get interior contours as well....
		//	contourFinder.findContours(grayImage, 10, (gKinect->width*gKinect->height)/2, 20, false);

		//	//for(int i = 0 ; i < contourFinder.nBlobs; i++)
		//	//{
		//	//	cout << "min: (" << contourFinder.blobs[i].boundingRect.x << ", " << contourFinder.blobs[i].boundingRect.x + contourFinder.blobs[i].boundingRect.y << ")" << endl;
		//	//	cout << "max: (" << contourFinder.blobs[i].boundingRect.x + contourFinder.blobs[i].boundingRect.width << ", " << contourFinder.blobs[i].boundingRect.x + contourFinder.blobs[i].boundingRect.y + contourFinder.blobs[i].boundingRect.height<< ")" << endl;
		//	//}

		//}
	}

	//--------------------------------------------------------------
	void OpenCV::draw() {
		/*
		ofResetColor();

		grayImage.draw(10, 320, 400, 300);

		//grayDiff.draw(420, 10, 400, 300);
		contourFinder.draw(10, 320, 400, 300);
		*/
		ofResetColor();
		stringstream reportStream;
		ofVec3f accel(gKinect->getMksAccel());
		float accelLength = accel.length();
		ofVec3f rotation(RAD_TO_DEG * acosf(accel.x / accelLength) - 90,
						RAD_TO_DEG * acosf(accel.y / accelLength),
						RAD_TO_DEG * acosf(accel.z / accelLength) - 90);

		reportStream << "accel is: "
			<< ofToString(gKinect->getMksAccel().x, 2) << " / "
			<< ofToString(gKinect->getMksAccel().y, 2) << " / " 
			<< ofToString(gKinect->getMksAccel().z, 2) << endl
			<< ofToString(rotation.x, 2) << " / "
			<< ofToString(rotation.y, 2) << " / " 
			<< ofToString(rotation.z, 2) << endl
			<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
			<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
			<< "set near threshold " << nearThreshold << " (press: + -)" << endl
			<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
			<< ", fps: " << ofGetFrameRate() << endl
			<< "press c to close the connection and o to open it again, connection is: " << gKinect->isConnected() << endl;
		ofDrawBitmapString(reportStream.str(),20,646);
	}

	//--------------------------------------------------------------
	void OpenCV::keyPressed (int key) {
		switch (key) {
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
		case '<':		
		case ',':		
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;

		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
		case '-':		
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
		}
	}

}