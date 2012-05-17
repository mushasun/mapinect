#include "bouncingball\BouncingBall.h"
#include "IMapinect.h"

//========================================================================
int main() {
	
	mapinect::IApplication* app = new bouncing::BouncingBall();

	mapinect::startMapinect(app);

	delete app;

	return 0;
}

