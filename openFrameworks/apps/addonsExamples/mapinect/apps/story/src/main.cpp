#include "story/Story.h"
#include "IMapinect.h"

//========================================================================
int main() {
	
	mapinect::IApplication* app = new story::Story();

	mapinect::startMapinect(app);

	delete app;

	return 0;
}

