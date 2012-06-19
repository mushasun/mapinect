#include "VisualizerApp.h"
#include "IMapinect.h"

//========================================================================
int main() {
	
	mapinect::IApplication* app = new visualizer::VisualizerApp();

	mapinect::startMapinect(app);

	delete app;

	return 0;
}

