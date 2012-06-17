#include "ColorBalancing.h"
#include "IMapinect.h"

//========================================================================
int main() {
	
	mapinect::IApplication* app = new colorBalancing::ColorBalancing();

	mapinect::startMapinect(app);

	delete app;

	return 0;
}

