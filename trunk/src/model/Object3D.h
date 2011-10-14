#ifndef MAPINECT_OBJECT3D_H__
#define MAPINECT_OBJECT3D_H__
#define MAX_FACES 3
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


#include <pcl/point_types.h>
#include "ofxVec3f.h"
#include "Quad3D.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include "ofxKinect.h"
#include "ofGraphicsUtils.h"

using namespace pcl;
using namespace mapinect;

class Object3D
{
	public:
		Object3D();
		Object3D(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ>::Ptr extendedCloud);
		void draw();
		inline PointCloud<PointXYZ> getCloud() { return cloud; }
		void updateCloud (PointCloud<PointXYZ>::Ptr nuCloud);
		~Object3D(void);
	private:
		Quad3D						faces[MAX_FACES];
		ofxVec3f					vMin;
		ofxVec3f					vMax;
		Eigen::Matrix4f				transformation;
		int							numFaces;
		PointCloud<PointXYZ>		cloud;
		PointCloud<PointXYZ>		extendedcloud;
		
		
		void detectFaces();

};

#endif	// MAPINECT_QUAD3D_H__