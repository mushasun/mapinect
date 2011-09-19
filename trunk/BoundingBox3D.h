#ifndef MAPINECT_BOUNDINGBOX3D_H__
#define MAPINECT_BOUNDINGBOX3D_H__

#include "ofMain.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "ofxVec3f.h"

using namespace pcl;

class BoundingBox3D
{
public:
	BoundingBox3D(PointCloud<PointXYZ>::ConstPtr blob);
	~BoundingBox3D(void);
	
	inline const ofxVec3f& getVA() { return v1; }
	inline const ofxVec3f& getVB() { return v2; }
	inline const ofxVec3f& getVC() { return v3; }
	inline const ofxVec3f& getVD() { return v4; }
	inline const ofxVec3f& getVE() { return v5; }
	inline const ofxVec3f& getVF() { return v6; }
	inline const ofxVec3f& getVG() { return v7; }
	inline const ofxVec3f& getVH() { return v8; }

private:
	ofxVec3f v1;
	ofxVec3f v2;
	ofxVec3f v3;
	ofxVec3f v4;
	ofxVec3f v5;
	ofxVec3f v6;
	ofxVec3f v7;
	ofxVec3f v8;

};

#endif	// MAPINECT_BOUNDINGBOX3D_H__