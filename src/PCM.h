#ifndef PCM_H__
#define PCM_H__

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/octree/octree.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>

#include "Quad3D.h"

#define KINECT_WIDTH 640
#define KINECT_HEIGHT 480
#define CLOUD_POINTS KINECT_WIDTH * KINECT_HEIGHT
#define OCTREE_RES 32 
#define MAX_PLANES 10
#define MIN_DIFF_TO_PROCESS 300

using namespace pcl;

class PCM {
public:

	virtual void setup(ofxKinect *kinect);
	virtual void update(bool isKinectFrameNew);
	virtual void draw();

	virtual void keyPressed(int key);
	virtual void mouseMoved(int x, int y);
	virtual void mouseDragged(int x, int y, int button);
	virtual void mousePressed(int x, int y, int button);
	virtual void mouseReleased(int x, int y, int button);
	virtual void windowResized(int w, int h);

	void drawPointCloud();

	void saveCloud(const string& name);
	void savePartialCloud(ofPoint min, ofPoint max, int id, const string& name);
	PointCloud<PointXYZ>* loadCloud(const string& name);

	void captureBlobsClouds();
	void detectPlanes(PointCloud<PointXYZ>::Ptr currentCloud);
	void processBlobsClouds();
	PointCloud<PointXYZ>::Ptr getPartialCloud(ofPoint min, ofPoint max);
	PointCloud<PointXYZ>::Ptr getCloud();
	PointCloud<PointXYZRGB>::Ptr getColorCloud();
	PointCloud<PointXYZRGB>::Ptr getPartialColorCloud(ofPoint min, ofPoint max);
	void setInitialPointCloud();
	PointCloud<PointXYZ>::Ptr getDifferenceIdx(bool &dif, int noise_filter = 7);
	void processDiferencesClouds();

	void icp();

	ofxKinect											*kinect;

	bool												drawPC;

	int 												pointCloudRotationY;

	pcl::PointCloud<pcl::PointXYZ>::Ptr					cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr					diffCloud;
	octree::OctreePointCloudChangeDetector<PointXYZ>	*octree;
	PointCloud<PointXYZ>								planes[MAX_PLANES];
	int													detectedPlanes;

	mapinect::Quad3D									detectedPlane;
	std::vector<ofxVec3f>								vCloudHull;

	bool												baseCloudSetted;
	float												timer;

};

#endif	// PCM_H__
