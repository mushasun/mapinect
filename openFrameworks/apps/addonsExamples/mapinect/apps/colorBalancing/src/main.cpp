#include "ColorBalancing\ColorBalancing.h"
#include "mapinect.h"

//========================================================================
int main() {
	
	mapinect::IApplication* app = new colorbalancing::ColorBalancing();

	mapinect::startMapinect(app);

	delete app;

	return 0;
}

