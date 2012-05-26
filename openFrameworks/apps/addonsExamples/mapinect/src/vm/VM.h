#ifndef VM_H__
#define VM_H__

#include <string>

namespace mapinect {

	class VM {
	public:
		virtual void setup();
		virtual void update();

		virtual void setupView();
		virtual void draw();
		virtual void endView();

		virtual void keyPressed(int key);
		virtual void mouseMoved(int x, int y );
		virtual void mouseDragged(int x, int y, int button);
		virtual void mousePressed(int x, int y, int button);
		virtual void mouseReleased(int x, int y, int button);
		virtual void windowResized(int w, int h);

		static std::string proj_calib_file;
		static std::string kinect_calib_file;

	private:
		bool		 isActive();
		virtual void loadProjCalibData(char* proj_calib_file);
		virtual void loadKinectExtrinsics(char* kinect_calib_file);
		virtual	void setProjMatrix(float fx, float fy, float cx, float cy);

	};
}

#endif	// VM_H__
