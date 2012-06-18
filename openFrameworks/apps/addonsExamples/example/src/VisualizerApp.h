#ifndef BUILDINGS_H__
#define BUILDINGS_H__

#include "Visualizer.h"

namespace visualizerApp {

	
	class VisualizerApp : public ofBaseApp {
	public:
		VisualizerApp();
		virtual ~VisualizerApp();

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

		virtual void debugDraw();

	private:
		void drawVisualizer();
        ofSoundPlayer music;

		Visualizer vis;
	};
}

#endif	// BUILDINGS_H__
