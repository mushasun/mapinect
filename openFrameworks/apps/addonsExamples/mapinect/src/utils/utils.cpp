#include "utils.h"

#include <sstream>

#include "Globals.h"
#include "Timer.h"

ofxKinect*				gKinect = NULL;
mapinect::Model*		gModel = NULL;
mapinect::Transformation*	gTransformation = NULL;


//Parsea array de floats separado por ',' sin espacios
std::vector<float> parseArray(const std::string& str)
{
	std::stringstream ss(str);
	std::vector<float> vect;
	float i;
	while (ss >> i)
	{
			vect.push_back(i);
			if (ss.peek() == ',')
					ss.ignore();
	}
	return vect;
}
