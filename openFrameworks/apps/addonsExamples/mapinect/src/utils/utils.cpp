#include "utils.h"

#include <sstream>

#include "Globals.h"
#include "Timer.h"

namespace mapinect
{
	float				OCTREE_RES;
	int					CLOUD_RES;
	float				CLOUD_RES_TO_VOXEL_FACTOR;
	float				MAX_Z;

	int					TIMES_TO_CREATE_OBJ;
	float				RES_IN_OBJ;
	float				RES_IN_OBJ2;
	float				MIN_DIF_PERCENT;
	int					MAX_OBJ_LOD;

	int					MIN_CLUSTER_SIZE;
	int					MAX_CLUSTER_SIZE;
	float				MAX_CLUSTER_TOLERANCE;

	int					MAX_TABLE_CLUSTER_SIZE;
	int					MIN_TABLE_CLUSTER_SIZE;
	float				MAX_TABLE_CLUSTER_TOLERANCE;

	float				TRANSLATION_DISTANCE_TOLERANCE;
	float				MAX_UNIFYING_DISTANCE;

	float				TOUCH_DISTANCE;

}

ofxKinect*				gKinect = NULL;
mapinect::Model*		gModel = NULL;


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
