#ifndef VISUALIZERAPP_H__
#define VISUALIZERAPP_H__

#include "IApplication.h"

#include <map>
#include "Floor.h"
#include "Box.h"
#include "Visualizer.h"

namespace visualizer {

	class VisualizerApp : public IApplication {
	public:
		VisualizerApp();
		virtual ~VisualizerApp();

		virtual void setup();
		virtual void update();
		virtual void draw();

		virtual void keyPressed(int key);

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
		vector<ofSoundPlayer>		tracks;
		int							currentTrack;
		
		float*						fftSmoothed;
		int							nBandsToGet;

		Visualizer					vis;
		ofImage*					btnLyric;

		ofImage*					btnPlay;
		ofImage*					btnPlayOn;
		ofImage*					btnNext;
		ofImage*					btnNextOn;
		ofImage*					btnPrev;
		ofImage*					btnPrevOn;

	};
}

#endif	// VISUALIZERAPP_H__
