#ifndef TEST_APP_H__
#define TEST_APP_H__

#include "ofMain.h"
#include "ofxFenster.h"

#include "OpenCV.h"
#include "PCM.h"
#include "VM.h"
#include "Arduino.h"
#include "Timer.h"

#include "IApplication.h"

namespace mapinect {

	class userApp;

	/// <summary>
	/// Mapinect core. This class is responsible of initializating globals and each module.
	/// It also drives the events to the modules.
	/// User's application is treated as a yet another module.
	/// </summary>
	class mapinectApp : public ofBaseApp {
	public:
		mapinectApp(ofxFenster* window, IApplication* app, VM* vm);
		virtual ~mapinectApp();

		/// <summary>
		/// Events for the primary window (debug drawing area)
		/// </summary>
		virtual void exit();
		virtual void setup();
		virtual void update();
		virtual void draw();

		virtual void keyPressed(int key);
		virtual void keyReleased(int key);
		virtual void windowMoved(int x, int y);
		virtual void mouseMoved(int x, int y);
		virtual void mouseDragged(int x, int y, int button);
		virtual void mousePressed(int x, int y, int button);
		virtual void mouseReleased(int x, int y, int button);
		virtual void dragEvent(ofDragInfo info);
		
	private:
		int				angle;

		/// <summary>
		/// Framework modules
		/// </summary>
		OpenCV			cv;
		PCM				pcm;
		Arduino			arduino;
		VM*				vm;
		ofxFenster*		window;

		/// <summary>
		/// User modules
		/// </summary>
		IApplication*	app;

	};

	class userApp : public ofxFensterListener
	{
	public:
		userApp(ofxFenster* window, IApplication* app);

		/// <summary>
		/// Events for the secondary window (user drawing area)
		/// </summary>
		virtual void exit();
		virtual void setup();
		virtual void update();
		virtual void draw();

		virtual void keyPressed(int key);
		virtual void keyReleased(int key);
		virtual void windowMoved(int x, int y);
		virtual void mouseMoved(int x, int y);
		virtual void mouseDragged(int x, int y, int button);
		virtual void mousePressed(int x, int y, int button);
		virtual void mouseReleased(int x, int y, int button);
		virtual void dragEvent(ofDragInfo info);

		mapinectApp*	mapinectAppPtr;
		VM*				vm;
		ofxFenster*		window;
		Timer			timer;

		/// <summary>
		/// User modules
		/// </summary>
		IApplication*	app;
	};
}

#endif	// MAPINECT_APP_H__
