#ifndef _TEST_APP
#define _TEST_APP
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/octree/octree.h>
#include <time.h>

#include "Quad3D.h"

#define KINECT_WIDTH 640
#define KINECT_HEIGHT 480
#define CLOUD_POINTS 307200 //640x480
#define OCTREE_RES 64 
#define MAX_PLANES 10
#define MIN_DIFF_TO_PROCESS 300

using namespace pcl;

class testApp : public ofBaseApp {
	public:

		void setup();
		void update();
		void draw();
		void exit();
	
		void drawPointCloud();

		void keyPressed  (int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);

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
		void printTime();

		ofxKinect kinect;

		ofxCvColorImage		colorImg;

		ofxCvGrayscaleImage 	grayImage;
		ofxCvGrayscaleImage 	grayThresh;
		ofxCvGrayscaleImage 	grayThreshFar;
		ofxCvGrayscaleImage 	grayBg;
		ofxCvGrayscaleImage 	grayDiff;

        ofxCvContourFinder 	contourFinder;

		int 				threshold;
		bool				bLearnBakground;
		
		bool				bThreshWithOpenCV;
		bool				drawPC;

		int 				nearThreshold;
		int					farThreshold;

		int					angle;
		
		int 				pointCloudRotationY;
		clock_t				start, end;

		pcl::PointCloud<pcl::PointXYZ>::Ptr	cloud;
		octree::OctreePointCloudChangeDetector<PointXYZ> *octree;
		PointCloud<PointXYZ>::Ptr planes[MAX_PLANES];

		mapinect::Quad3D		detectedPlane;
		std::vector<ofxVec3f>	vCloudHull;

		bool				baseCloudSetted;
		float				timer;
};

#endif
