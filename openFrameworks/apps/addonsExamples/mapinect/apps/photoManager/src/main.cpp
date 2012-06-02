#include "PhotoManager.h"
#include "IMapinect.h"

//========================================================================
int main() {
	
	mapinect::IApplication* app = new photo::PhotoManager();

	mapinect::startMapinect(app);

	delete app;

	return 0;
}

