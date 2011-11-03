#ifndef UTILS_H__
#define UTILS_H__

#define KINECT_WIDTH 640
#define KINECT_HEIGHT 480

#include "ofxKinect.h"
#include "Model.h"

extern ofxKinect			*gKinect;
extern mapinect::Model		*gModel;

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

extern string				ARDUINO_COM_PORT;

extern int					objId;

#define MAX_FLOAT		FLT_MAX

inline double dabsd(double value) {
	if (value < 0) {
		return -value;
	}
	else {
		return value;
	}
}

#endif	// UTILS_H__