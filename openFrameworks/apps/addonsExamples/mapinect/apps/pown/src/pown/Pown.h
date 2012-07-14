#ifndef POWN_H__
#define POWN_H__

#include "IApplication.h"

#include <map>
#include "Box.h"

namespace pown
{
	class Pown : public IApplication
	{
	public:
		Pown();
		virtual ~Pown();

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

	};
}

#endif	// POWN_H__
