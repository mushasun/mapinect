#ifndef DRAWING_H__
#define DRAWING_H__

#include "IApplication.h"

#include <map>
#include "Canvas.h"

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

	private:
		map<int, map<int, Canvas*> >		canvas;
		map<int, DataTouch>					touchPoints;

		ofColor								backColor;
		ofColor								foreColor;
	};
}

#endif	// DRAWING_H__
