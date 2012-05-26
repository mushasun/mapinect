#ifndef MAPINECT_CONSTANTS_H__
#define MAPINECT_CONSTANTS_H__

#include <vector>

using namespace std;

namespace mapinect {
	
	extern float				OCTREE_RES;
	extern int					MIN_DIFF_TO_PROCESS;
	extern int					QUAD_HALO;
	extern int					DIFF_THRESHOLD;
	extern float				RES_IN_OBJ;
	extern float				RES_IN_OBJ2;
	extern int					DIFF_IN_OBJ;
	extern int					TIMES_TO_CREATE_OBJ;
	extern float				MIN_DIF_PERCENT;
	extern int					TIMES_TO_STABILIZE;
	extern float				MAX_Z;
	extern float				NORMAL_ESTIMATION_PERCENT;
	extern int					MAX_CLUSTER_SIZE;
	extern int					MIN_CLUSTER_SIZE;
	extern float				MAX_CLUSTER_TOLERANCE;
	extern int					MAX_TABLE_CLUSTER_SIZE;
	extern int					MIN_TABLE_CLUSTER_SIZE;
	extern float				MAX_TABLE_CLUSTER_TOLERANCE;
	extern float				TRANSLATION_DISTANCE_TOLERANCE;
	extern int					CLOUD_RES;
	extern int					MAX_OBJ_LOD;
	extern float				MAX_UNIFYING_DISTANCE;
	extern int					KINECT_WIDTH;
	extern int					KINECT_HEIGHT;
	extern int					KINECT_WIDTH_OFFSET;
	extern int					KINECT_HEIGHT_OFFSET;
	extern float				MAX_UNIFYING_DISTANCE_PROJECTION;
	extern float				TOUCH_DISTANCE;
	extern float				HAND_SIZE;
	extern vector<float>		MIN_ANGLES_FINGERS;
	extern vector<float>		MAX_ANGLES_FINGERS;
	extern vector<float>		MIN_LENGTH_FINGERS;
	extern vector<float>		MAX_LENGTH_FINGERS;

}

#endif	// MAPINECT_CONSTANTS_H__
