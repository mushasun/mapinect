#include "ofMain.h"
#include "ofAppGlutWindow.h"
#include "armCalibration/ArmCalibration.h"

//========================================================================
int main() {
	
	// create an opengl window
    ofAppGlutWindow window;
	
	// setup the window at a given size in normal (non-fullcreen mode )
	// use OF_FULLSCREEN if you want to change the window mode
	ofSetupOpenGL(&window, 800, 600, OF_WINDOW);			

	// create an instance of the armCalibration and run it as our main application
	ofRunApp(new armCalibration::ArmCalibration());

	return 0;
}

