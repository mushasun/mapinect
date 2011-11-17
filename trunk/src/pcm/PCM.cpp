#include "PCM.h"

#include "utils.h"

#define DEFAULT_NAME		"test"

using namespace std;

namespace mapinect {
	//--------------------------------------------------------------
	void PCM::setup() {
		pcmThread.setup();
		drawPC = false;
	}

	//--------------------------------------------------------------
	void PCM::exit() {
		pcmThread.exit();
	}

	//--------------------------------------------------------------
	void PCM::update(bool isKinectFrameNew) {

	}

	//--------------------------------------------------------------
	void PCM::draw() {
		ofResetColor();

		if(drawPC) {
			ofPushMatrix();
			ofTranslate(420, 320);
			// we need a proper camera class
			drawPointCloud();
			ofPopMatrix();
		}
		else {
			gKinect->drawDepth(0, 0, 640, 480);
			ofResetColor();
			ofPushMatrix();
			gModel->objectsMutex.lock();
			/*glTranslatef(320, 240, 0);
			glScalef(1, 1, 1);*/
			for (list<ModelObject*>::iterator iter = gModel->objects.begin();
				iter != gModel->objects.end(); iter++) {
					(*iter)->drawObject();
			}
			if(gModel->table != NULL)
				gModel->table->draw();	
			gModel->objectsMutex.unlock();

			ofPopMatrix();
		}
	}

	//--------------------------------------------------------------
	void PCM::drawPointCloud() {
		ofScale(400, 400, 400);
		int w = 640;
		int h = 480;

		ofRotateY(pointCloudRotationY);
		float* distancePixels = gKinect->getDistancePixels();

		glBegin(GL_POINTS);
		int step = 2;
		for(int y = 0; y < h; y += step) {
			for(int x = 0; x < w; x += step) {
				ofPoint cur = gKinect->getWorldCoordinateFor(x, y);
				ofColor color = gKinect->getCalibratedColorAt(x,y);
				glColor3ub((unsigned char)color.r,(unsigned char)color.g,(unsigned char)color.b);
				glVertex3f(cur.x, cur.y, cur.z);
			}
		}
		glEnd();

	}

	//--------------------------------------------------------------
	void PCM::keyPressed (int key) {
		switch (key) {
		case ' ':
			pcmThread.detectMode = !pcmThread.detectMode;
			break;
		case'p':
			drawPC = !drawPC;
			break;
		//case 's':
		//	saveCloud(DEFAULT_NAME);
		//	break;
		case 'd':
			pcmThread.processDiferencesClouds();
			break;
		case 'r':
			pcmThread.reset();
			gModel->objectsMutex.lock();
			gModel->objects.clear();	
			gModel->objectsMutex.unlock();
			break;
		}
		//case 't':
		//	setTransformation();
		//	break;
	}

}
