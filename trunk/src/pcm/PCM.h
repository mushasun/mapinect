#ifndef PCM_H__
#define PCM_H__

#include "PCMThread.h"

namespace mapinect {
	class PCM {
	public:

		virtual void setup();
		virtual void update(bool isKinectFrameNew);
		virtual void draw();

		virtual void keyPressed(int key);
		virtual void mouseMoved(int x, int y);
		virtual void mouseDragged(int x, int y, int button);
		virtual void mousePressed(int x, int y, int button);
		virtual void mouseReleased(int x, int y, int button);
		virtual void windowResized(int w, int h);

		void drawPointCloud();

		bool												drawPC;

		int 												pointCloudRotationY;

	private:
		PCMThread											pcmThread;

	};
}

#endif	// PCM_H__
