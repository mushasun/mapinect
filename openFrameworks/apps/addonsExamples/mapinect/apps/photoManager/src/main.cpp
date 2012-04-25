#include "photomanager\PhotoManager.h"
#include "mapinect.h"

//========================================================================
int main() {
	
	mapinect::IApplication* app = new photo::PhotoManager();

	mapinect::startMapinect(app);

	delete app;

	return 0;
}

