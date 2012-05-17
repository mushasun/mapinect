#ifndef TEST_APP_H__
#define TEST_APP_H__

#include "ofMain.h"
#include "ofxFenster.h"

#include "OpenCV.h"
#include "PCM.h"
#include "VM.h"
#include "Arduino.h"

#include "IApplication.h"

namespace mapinect {

	/// <summary>
	/// Mapinect core. This class is responsible of initializating globals and each module.
	/// It also drives the events to the modules.
	/// User's application is treated as a yet another module.
	/// </summary>
	class mapinectApp : public ofBaseApp, public ofxFensterListener {
	public:
		mapinectApp(IApplication* app);
		virtual ~mapinectApp();

		/// <summary>
		/// Events for the primary window (debug drawing area)
		/// </summary>
		virtual void setup();
		virtual void update();
		virtual void draw();
		virtual void exit();

		virtual void keyPressed(int key);
		virtual void mouseMoved(int x, int y );
		virtual void mouseDragged(int x, int y, int button);
		virtual void mousePressed(int x, int y, int button);
		virtual void mouseReleased(int x, int y, int button);
		virtual void windowResized(int w, int h);
		
		/// <summary>
		/// Events for the secondary window (user drawing area)
		/// </summary>
		virtual void fensterUpdate();
		virtual void fensterSetup(); // Fenster setup added
		virtual void fensterDraw();

		virtual void fensterKeyPressed(int key);
		virtual void fensterMouseMoved(int x, int y );
		virtual void fensterMouseDragged(int x, int y, int button);
		virtual void fensterMousePressed(int x, int y, int button);
		virtual void fensterMouseReleased(int x, int y, int button);
		virtual void fensterWindowResized(int w, int h);

		int				angle;

		/// <summary>
		/// Framework modules
		/// </summary>
		OpenCV			cv;
		PCM				pcm;
		VM				vm;
		Arduino			arduino;

		/// <summary>
		/// User modules
		/// </summary>
		IApplication*	app;

	private:
		void getKinectCalibData(char* kinect_calib_file, double& d_fx, double& d_fy, float& d_cx, float& d_cy,
									double& rgb_fx, double& rgb_fy, float& rgb_cx, float& rgb_cy,
									ofVec3f& T, ofMatrix4x4& R);

	};
}

#endif	// TEST_APP_H__
