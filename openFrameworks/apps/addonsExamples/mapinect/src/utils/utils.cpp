#include "utils.h"

#include <sstream>

#include "Globals.h"

namespace mapinect
{
	float				OCTREE_RES;
	int					MIN_DIFF_TO_PROCESS;
	int					QUAD_HALO;
	int					DIFF_THRESHOLD;
	float				RES_IN_OBJ;
	float				RES_IN_OBJ2;
	int					DIFF_IN_OBJ;
	int					TIMES_TO_CREATE_OBJ;
	float				MIN_DIF_PERCENT;
	int					TIMES_TO_STABILIZE;
	float				MAX_Z;
	float				NORMAL_ESTIMATION_PERCENT;
	int					MAX_CLUSTER_SIZE;
	int					MIN_CLUSTER_SIZE;
	float				MAX_CLUSTER_TOLERANCE;
	int					MAX_TABLE_CLUSTER_SIZE;
	int					MIN_TABLE_CLUSTER_SIZE;
	float				MAX_TABLE_CLUSTER_TOLERANCE;
	int					CLOUD_RES;
	float				TRANSLATION_DISTANCE_TOLERANCE;
	int					MAX_OBJ_LOD;
	int					objId;
	float				MAX_UNIFYING_DISTANCE;
	int					KINECT_WIDTH;
	int					KINECT_HEIGHT;
	int					KINECT_WIDTH_OFFSET;
	int					KINECT_HEIGHT_OFFSET;
	float				MAX_UNIFYING_DISTANCE_PROJECTION;
	float				TOUCH_DISTANCE;
	float				HAND_SIZE;
	std::vector<float>	MIN_ANGLES_FINGERS;
	std::vector<float>	MAX_ANGLES_FINGERS;
	std::vector<float>	MIN_LENGTH_FINGERS;
	std::vector<float>	MAX_LENGTH_FINGERS;
}

int						objId = 0;
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
