#ifndef LPMT_H__
#define LPMT_H__

#define OF_ADDON_USING_OFXXMLSETTINGS

#include "ofxKinect.h"
#include "ball.h"
#include "quad.h"
#include "ofxXmlSettings.h"

class LPMT {
public:
	void setup(ofxKinect *kinect);
	void update();
	void draw();

	void keyPressed(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);

	void setXml();
	void getXml();

	ofxKinect* kinect;

	int whichCorner;
	ofTrueTypeFont ttf;

	quad quads[36];
	int layers[36];

	int activeQuad;
	int nOfQuads;
	int borderColor;

	bool isSetup;
	bool bFullscreen;
	bool bGui;
	bool snapshotOn;

	// gui elements
	bool showGui;

	// camera grabber
	ofVideoGrabber camGrabber;
	ofTexture camTexture;
	ofTexture snapshotTexture;

	int camWidth;
	int camHeight;

	vector<string> imgFiles;
	vector<string> videoFiles;
	vector<string> slideshowFolders;

	ofxXmlSettings XML;
};

#endif	// LPMT_H__
