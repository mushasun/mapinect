#ifndef UTILS_H__
#define UTILS_H__



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
extern int					KINECT_WIDTH;
extern int					KINECT_HEIGHT;
extern int					KINECT_WIDTH_OFFSET;
extern int					KINECT_HEIGHT_OFFSET;
extern float				MAX_UNIFYING_DISTANCE_PROJECTION;

extern int					objId;

#define MAX_FLOAT		FLT_MAX

struct ofPolar {
	float ro, theta;
};

inline ofxVec3f scaleFromMtsToMms(ofxVec3f p)
{
	ofxVec3f res;
	// Transform from meters to milimeters
	res.x = (p.x)*1000;
	res.y = (p.y)*1000;
	res.z = (p.z)*1000;
	return res;	
}
ofPolar cartesianToPolar(const ofPoint& c);

bool sortOnY(ofxVec3f l, ofxVec3f r);

bool sortOnX(ofxVec3f l, ofxVec3f r);

bool sortOnZ(ofxVec3f l, ofxVec3f r);

#endif	// UTILS_H__