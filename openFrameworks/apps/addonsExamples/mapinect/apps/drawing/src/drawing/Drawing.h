#ifndef DRAWING_H__
#define DRAWING_H__

#include "IApplication.h"

#include <map>
#include <vector>
#include "ofSoundPlayer.h"

using namespace mapinect;

namespace drawing
{
	class Drawing : public IApplication
	{
	public:
		Drawing();
		virtual ~Drawing();

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

		virtual void objectDetected(const IObjectPtr&);
		virtual void objectUpdated(const IObjectPtr&);
		virtual void objectLost(const IObjectPtr&);
		virtual void objectMoved(const IObjectPtr&, const DataMovement&);
		virtual void objectTouched(const IObjectPtr&, const DataTouch&);

	private:
		std::map<int, DataTouch>	touchPoints;
		vector<ofSoundPlayer>		sounds;
	};
}

#endif	// DRAWING_H__
