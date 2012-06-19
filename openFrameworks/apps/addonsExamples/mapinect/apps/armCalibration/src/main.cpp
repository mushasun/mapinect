#include "armCalibration/ArmCalibration.h"

//========================================================================
int main() {
	
	mapinect::IApplication* app = new armCalibration::ArmCalibration();

	mapinect::startMapinect(app);

	delete app;

	return 0;
}

