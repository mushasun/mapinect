#ifndef ARMCALIBRATION_H__
#define ARMCALIBRATION_H__

#include "ofMain.h"

#include "Arduino.h"
#include "mapinectTypes.h"

using namespace mapinect;

namespace armCalibration
{
	struct MotorRotationsCloud
	{
		MotorRotationsCloud(signed int* rotations, const PCPtr& cloud)
			: rotations(rotations), cloud(cloud)
		{
		}
	public:
		signed int*		rotations;
		PCPtr			cloud;
	};

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

		void storeCloud();
		void saveRawClouds();
		void loadRawClouds();
		void saveTransformedClouds();

		void changeValue(float& value, float delta);

	private:
		Arduino		arduino;

		vector<MotorRotationsCloud>		storedClouds;
	};
}

#endif	// ARMCALIBRATION_H__
