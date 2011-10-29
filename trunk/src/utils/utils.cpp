#include "utils.h"

ofxKinect			*gKinect = 0;
mapinect::Model	*gModel = 0;

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
int					objId;
