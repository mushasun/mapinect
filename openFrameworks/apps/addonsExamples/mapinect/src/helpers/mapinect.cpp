#define NOMINMAX

#include "IMapinect.h"

#include "ofMain.h"
#include "mapinectApp.h"
#include "ofAppGlutWindow.h"
#include "ofxFensterManager.h"

namespace mapinect {
	void startMapinect(IApplication* app) {

		ofSetupOpenGL(ofxFensterManager::get(), 800, 600, OF_WINDOW);			// <-------- setup the GL context

		// this kicks off the running of my app
		// can be OF_WINDOW or OF_FULLSCREEN
		// pass in width and height too:
		ofRunFensterApp(new userApp(ofxFensterManager::get()->getPrimaryWindow(), app));

	}
}
