#ifndef MAPINECT_CONSTANTS_H__
#define MAPINECT_CONSTANTS_H__

#include <vector>

using namespace std;

namespace mapinect {
	
	extern float				OCTREE_RES;
	extern int					CLOUD_RES;
	extern float				CLOUD_RES_TO_VOXEL_FACTOR;
	extern float				MAX_Z;

	extern int					TIMES_TO_CREATE_OBJ;
	extern float				RES_IN_OBJ;
	extern float				RES_IN_OBJ2;
	extern float				MIN_DIF_PERCENT;
	extern int					MAX_OBJ_LOD;

	extern int					MIN_CLUSTER_SIZE;
	extern int					MAX_CLUSTER_SIZE;
	extern float				MAX_CLUSTER_TOLERANCE;

	extern int					MAX_TABLE_CLUSTER_SIZE;
	extern int					MIN_TABLE_CLUSTER_SIZE;
	extern float				MAX_TABLE_CLUSTER_TOLERANCE;

	extern float				TRANSLATION_DISTANCE_TOLERANCE;
	extern float				MAX_UNIFYING_DISTANCE;

	extern float				TOUCH_DISTANCE;

}

#endif	// MAPINECT_CONSTANTS_H__
