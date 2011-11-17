#ifndef I_APPLICATION_H__
#define I_APPLICATION_H__

#include "ITxManager.h"

class IApplication {
public:

	virtual void setup() = 0;
	virtual void update() = 0;
	virtual void draw() = 0;
	virtual void exit() = 0;

	virtual void keyPressed(int key) = 0;
	virtual void mouseMoved(int x, int y) = 0;
	virtual void mouseDragged(int x, int y, int button) = 0;
	virtual void mousePressed(int x, int y, int button) = 0;
	virtual void mouseReleased(int x, int y, int button) = 0;
	virtual void windowResized(int w, int h) = 0;

	mapinect::ITxManager*	txManager;
};

#endif	// I_APPLICATION_H__
