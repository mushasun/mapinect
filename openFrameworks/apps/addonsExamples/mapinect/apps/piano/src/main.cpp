#include "Pown.h"
#include "IMapinect.h"

//========================================================================
int main() {
	
	mapinect::IApplication* app = new pown::Pown();

	mapinect::startMapinect(app);

	delete app;

	return 0;
}

