#include "Buildings.h"
#include "IMapinect.h"

//========================================================================
int main() {
	
	mapinect::IApplication* app = new buildings::Buildings();

	mapinect::startMapinect(app);

	delete app;

	return 0;
}

