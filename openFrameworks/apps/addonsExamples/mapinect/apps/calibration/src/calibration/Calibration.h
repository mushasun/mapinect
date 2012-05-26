#ifndef CALIBRATION_H__
#define CALIBRATION_H__

#include "IApplication.h"

#include "ofImage.h"
#include "ofBaseTypes.h"

namespace calibration
{
	class Calibration : public mapinect::IApplication
	{
	public:
		virtual void setup();
		virtual void exit();
		virtual void update();
		virtual void draw();

		virtual void keyPressed(int key);
		virtual void mouseMoved(int x, int y );
		virtual void mouseDragged(int x, int y, int button);
		virtual void mousePressed(int x, int y, int button);
		virtual void mouseReleased(int x, int y, int button);
		virtual void windowResized(int w, int h);

		virtual void debugDraw();
	
	private:
		ofImage		pattern;

	};
}

#endif	// CALIBRATION_H__
