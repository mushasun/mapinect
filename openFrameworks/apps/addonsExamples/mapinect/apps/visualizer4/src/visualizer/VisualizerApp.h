#ifndef VISUALIZERAPP_H__
#define VISUALIZERAPP_H__

#include "IApplication.h"

#include "Floor.h"
#include "Box.h"
#include <map>
#include "ofMain.h"
#include "Visualizer.h"
#include "ButtonManager.h"

namespace visualizer {

	class VisualizerApp : public IApplication {
	public:
		VisualizerApp();
		virtual ~VisualizerApp();

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

		virtual void objectDetected(const IObjectPtr&);
		virtual void objectUpdated(const IObjectPtr&);
		virtual void objectLost(const IObjectPtr&);
		virtual void objectMoved(const IObjectPtr&, const DataMovement&);
		virtual void objectTouched(const IObjectPtr&, const DataTouch&);
		virtual void buttonPressed(const IButtonPtr&);
		virtual void buttonReleased(const IButtonPtr&);

	private:
		std::map<int, Box*>	boxes;
		std::map<int, DataTouch>	touchPoints;
		Floor*						floor;
		ofSoundPlayer				music;
		ofSoundPlayer				ding;
		ofSoundPlayer				dong;
		ofSoundPlayer				sample1;
		ofSoundPlayer				sample2;
		ofSoundPlayer				sample3;
		ofSoundPlayer				sample4;

		float*						fftSmoothed;
		int							nBandsToGet;

		Visualizer					vis;
	};
}

#endif	// VISUALIZERAPP_H__
