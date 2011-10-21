#ifndef UTILS_H__
#define UTILS_H__

#define KINECT_WIDTH 640
#define KINECT_HEIGHT 480

#include "ofxKinect.h"
#include "Model.h"

extern ofxKinect			*gKinect;
extern mapinect::Model	*gModel;

extern float				OCTREE_RES;
extern int					MIN_DIFF_TO_PROCESS;
extern int					QUAD_HALO;
extern int					DIFF_THRESHOLD;
extern float				RES_IN_OBJ;
extern int					DIFF_IN_OBJ;
extern int					TIMES_TO_CREATE_OBJ;


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