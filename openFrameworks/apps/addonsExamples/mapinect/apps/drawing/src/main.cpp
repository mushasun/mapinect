#include "Drawing.h"
#include "IMapinect.h"

//========================================================================
int main() {
	
	mapinect::IApplication* app = new drawing::Drawing();

	mapinect::startMapinect(app);

	delete app;

	return 0;
}

