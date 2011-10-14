#include "utils.h"

ofxKinect			*gKinect = 0;
mapinect::Model3D	*gModel3D = 0;

float				OCTREE_RES;
int					MIN_DIFF_TO_PROCESS;
int					QUAD_HALO;
int					DIFF_THRESHOLD;
float				RES_IN_OBJ;
int					DIFF_IN_OBJ;
int					TIMES_TO_CREATE_OBJ;
