#ifndef ARMCALIBRATION_H__
#define ARMCALIBRATION_H__

#include "Arduino.h"

using namespace mapinect;

namespace calibration
{
	class ArmCalibration : public ofBaseApp
	{
	public:
		virtual void setup();
		virtual void update();
		virtual void draw();
		virtual void exit();

		virtual void keyPressed(int key);
		virtual void keyReleased(int key);
		virtual void windowMoved(int x, int y);
		virtual void mouseMoved(int x, int y);
		virtual void mouseDragged(int x, int y, int button);
		virtual void mousePressed(int x, int y, int button);
		virtual void mouseReleased(int x, int y, int button);
		virtual void dragEvent(ofDragInfo info);

		virtual void debugDraw();

	private:
		Arduino		arduino;

		ofVec3f		armLength;
		ofVec3f		motor84;
		ofVec3f		
	};
}

#endif	// CALIBRATION_H__
