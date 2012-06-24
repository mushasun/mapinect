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
		virtual void draw();

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
