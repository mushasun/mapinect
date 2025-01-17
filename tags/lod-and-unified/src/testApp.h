#ifndef TEST_APP_H__
#define TEST_APP_H__

#include "ofMain.h"
#include "ofxFenster.h"
#include "monitor.h"
#include "winUtils.h"

#include "OpenCV.h"
#include "PCM.h"
#include "LPMT.h"
#include "Arduino.h"

class testApp : public ofBaseApp, public ofxFensterListener {
public:

	virtual void setup();
	virtual void update();
	virtual void draw();
	virtual void exit();

	virtual void keyPressed(int key);
	virtual void mouseMoved(int x, int y );
	virtual void mouseDragged(int x, int y, int button);
	virtual void mousePressed(int x, int y, int button);
	virtual void mouseReleased(int x, int y, int button);
	virtual void windowResized(int w, int h);

	virtual void fensterUpdate();
	virtual void fensterDraw();
	virtual void fensterKeyPressed(int key);
	virtual void fensterMouseMoved(int x, int y );
	virtual void fensterMouseDragged(int x, int y, int button);
	virtual void fensterMousePressed(int x, int y, int button);
	virtual void fensterMouseReleased(int x, int y, int button);
	virtual void fensterWindowResized(int w, int h);

	int						angle;

	mapinect::OpenCV		cv;
	mapinect::PCM			pcm;
	mapinect::LPMT			lpmt;

	signed int valorArduino1;
	signed int valorArduino2;
	signed int valorArduino4;
	signed int valorArduino8;
	Arduino* ard;
};

#endif	// TEST_APP_H__
