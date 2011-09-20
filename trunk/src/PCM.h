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
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include "ofxXmlSettings.h"
#include "Quad3D.h"
#include "Object3D.h"
#include "OpenCV.h"

#define KINECT_WIDTH 640
#define KINECT_HEIGHT 480
#define CLOUD_POINTS KINECT_WIDTH * KINECT_HEIGHT
#define MAX_OBJECTS 10

using namespace pcl;

class PCM {
public:

	virtual void setup(ofxKinect *kinect, OpenCV *openCVService);
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
	PointCloud<PointXYZ>::Ptr getPartialCloudRealCoords(ofPoint min, ofPoint max, int density = 4);
	PointCloud<PointXYZ>::Ptr getCloud();
	PointCloud<PointXYZRGB>::Ptr getColorCloud();
	
	PointCloud<PointXYZRGB>::Ptr getPartialColorCloud(ofPoint min, ofPoint max);
	void setInitialPointCloud();
	PointCloud<PointXYZ>::Ptr getDifferenceIdx(bool &dif, int noise_filter = 7);
	void processDiferencesClouds();
	void setTransformation();

	void icp();

	ofxKinect											*kinect;

	bool												drawPC;

	int 												pointCloudRotationY;

	pcl::PointCloud<pcl::PointXYZ>::Ptr					cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr					currentDiffcloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr					diffCloud;
	octree::OctreePointCloudChangeDetector<PointXYZ>	*octree;
	
	bool												baseCloudSetted;
	float												timer;
	Eigen::Matrix4f										transformation;

	Object3D											detectedObjects[MAX_OBJECTS];
	bool												updateDetectedObject(PointCloud<PointXYZ>::Ptr cloud_cluster);
private:
	ofxVec3f normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr plane);
	ofxVec3f normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr plane, pcl::PointIndices::Ptr indicesptr);
	int													getSlotForTempObj();
	int													noDifferencesCount;
	OpenCV												*openCVService;
	int													nDetectedObjects;
	bool												detectMode;

	PointCloud<PointXYZ>								tempObjects[MAX_OBJECTS];
	int													tempObjectsStatus[MAX_OBJECTS];
	//Propiedades cargadas de XML
	float												OCTREE_RES;
	int													MIN_DIFF_TO_PROCESS;
	int													QUAD_HALO;
	int													DIFF_THRESHOLD;
	float												RES_IN_OBJ;
	int													DIFF_IN_OBJ;
	int													TIMES_TO_CREATE_OBJ;
};

#endif	// PCM_H__
