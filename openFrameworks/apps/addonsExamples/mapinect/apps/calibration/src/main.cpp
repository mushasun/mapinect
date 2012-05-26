#include "Calibration/Calibration.h"
#include "IMapinect.h"

//========================================================================
int main() {
	
	mapinect::IApplication* app = new calibration::Calibration();

	mapinect::startMapinect(app);

	delete app;

	return 0;
}

