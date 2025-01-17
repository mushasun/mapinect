#define NOMINMAX

#include "ofMain.h"
#include "testApp.h"
#include "ofAppGlutWindow.h"
#include "ofxFenster.h"

//========================================================================
int main( ){

    ofAppGlutWindow window;
	ofSetupOpenGL(&window, 1024,768, OF_WINDOW);			// <-------- setup the GL context

	testApp* app = new testApp;
	
	ofxFenster fenster;
	fenster.init(app, "mapinect_aux");

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(app);

}

