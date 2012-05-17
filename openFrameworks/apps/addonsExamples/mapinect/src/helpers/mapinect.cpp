#define NOMINMAX

#include "IMapinect.h"

#include "ofMain.h"
#include "mapinectApp.h"
#include "ofAppGlutWindow.h"
#include "ofxFenster.h"

namespace mapinect {
	void startMapinect(IApplication* app) {

		ofAppGlutWindow window;
		ofSetupOpenGL(&window, 1024,768, OF_WINDOW);			// <-------- setup the GL context

		mapinectApp* mapApp = new mapinectApp(app);
	
		ofxFenster fenster;
		fenster.init(mapApp, "mapinect_aux");

		// this kicks off the running of my app
		// can be OF_WINDOW or OF_FULLSCREEN
		// pass in width and height too:
		ofRunApp(mapApp);

	}
}
