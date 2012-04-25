#ifndef OPEN_CV_H__
#define OPEN_CV_H__

#include "ofxOpenCv.h"

namespace mapinect {
	class OpenCV {
	public:
		virtual void setup();
		virtual void exit();
		virtual void update(bool isKinectFrameNew);
		virtual void draw();

		virtual void keyPressed(int key);

	private:
		ofxCvColorImage			colorImg;

		ofxCvGrayscaleImage 	grayImage;
		ofxCvGrayscaleImage 	grayThresh;
		ofxCvGrayscaleImage 	grayThreshFar;
		ofxCvGrayscaleImage 	grayBg;
		ofxCvGrayscaleImage 	grayDiff;

		ofxCvContourFinder 		contourFinder;

		int 					threshold;

		bool					bThreshWithOpenCV;
		bool					drawPC;
		bool					drawCalibration;

		int 					nearThreshold;
		int						farThreshold;

		bool					drawDepth;
	};
}

#endif	// OPEN_CV_H__
