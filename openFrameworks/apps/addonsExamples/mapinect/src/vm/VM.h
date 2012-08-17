#ifndef VM_H__
#define VM_H__

#include <string>
#include "ofVec3f.h"
#include <Eigen/Geometry>
#include "ofxOpenCv.h"

namespace mapinect {

	class VM {
	public:
		virtual void setup();
		virtual void update();

		virtual void setupView();
		virtual void draw();
		virtual void endView();

		virtual void keyPressed(int key);
		virtual void keyReleased(int key);
		virtual void windowMoved(int x, int y);
		virtual void mouseMoved(int x, int y);
		virtual void mouseDragged(int x, int y, int button);
		virtual void mousePressed(int x, int y, int button);
		virtual void mouseReleased(int x, int y, int button);
		virtual void dragEvent(ofDragInfo info);

		virtual std::string getKinectCalibFile();

	private:
		bool		 isActive();
		virtual void loadProjCalibData(char* projCalibFile);
		virtual void loadKinectExtrinsics(char* kinectCalibFile);
		
		virtual	void setProjMatrix(float fx, float fy, float cx, float cy);

		virtual float*	getTransformationMatrixOpenGL(CvMat* R, CvMat* T);
		virtual float*	getInverseTransformationMatrixOpenGL(CvMat* R, CvMat* T);

		virtual void loadCalibParams();

		static std::string projCalibFile;
		static std::string kinectCalibFile;

		static float projWidth;
		static float projHeight;

		static float nearPlane;
		static float farPlane;

	};


	//--------------------------------------------------------------
	void getKinectCalibData(const string& kinect_calib_file,
							double& d_fx, double& d_fy, float& d_cx, float& d_cy,
							double& rgb_fx, double& rgb_fy, float& rgb_cx, float& rgb_cy,
							ofVec3f& T, ofMatrix4x4& R);
}

#endif	// VM_H__
