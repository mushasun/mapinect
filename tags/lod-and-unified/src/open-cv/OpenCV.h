#ifndef OPEN_CV_H__
#define OPEN_CV_H__

#include "ofxOpenCv.h"
#include "ofxKinect.h"

namespace mapinect {
	class OpenCV {
	public:
		virtual void setup();
		virtual void update(bool isKinectFrameNew);
		virtual void draw();

		virtual void keyPressed(int key);
		virtual void mouseMoved(int x, int y );
		virtual void mouseDragged(int x, int y, int button);
		virtual void mousePressed(int x, int y, int button);
		virtual void mouseReleased(int x, int y, int button);
		virtual void windowResized(int w, int h);

		ofxCvColorImage			colorImg;

		ofxCvGrayscaleImage 	grayImage;
		ofxCvGrayscaleImage 	grayThresh;
		ofxCvGrayscaleImage 	grayThreshFar;
		ofxCvGrayscaleImage 	grayBg;
		ofxCvGrayscaleImage 	grayDiff;

		ofxCvContourFinder 		contourFinder;

		int 				threshold;

		bool				bThreshWithOpenCV;
		bool				drawPC;
		bool				drawCalibration;

		int 				nearThreshold;
		int					farThreshold;

		bool				drawDepth;
	};
}

#endif	// OPEN_CV_H__
