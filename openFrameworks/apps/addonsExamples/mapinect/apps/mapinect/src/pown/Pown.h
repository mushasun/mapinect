#ifndef POWN_H__
#define POWN_H__

#include "IApplication.h"

#include <set>
#include <map>
#include "Brick.h"
#include "Box.h"
#include "Light.h"

namespace pown
{
	class Pown : public IApplication
	{
	public:
		Pown();
		virtual ~Pown();

		virtual void		setup();
		virtual void		update(float elapsedTime);
		virtual void		draw();

		virtual void		objectDetected(const IObjectPtr&);
		virtual void		objectUpdated(const IObjectPtr&);
		virtual void		objectLost(const IObjectPtr&);
		virtual void		objectMoved(const IObjectPtr&, const DataMovement&);
		virtual void		objectTouched(const IObjectPtr&, const DataTouch&);
		virtual void		buttonPressed(const IButtonPtr&, const DataTouch&);
		virtual void		buttonReleased(const IButtonPtr&, const DataTouch&);
		virtual void		pointTouched(const DataTouch&);

		virtual void		keyPressed(int key);

	private:
		void				updateBeat(float elapsedTime);

		IPolygonPtr			floor;
		BrickManager*		brickManager;
		map<int, Box*>		boxes;

		Light				light;
		bool				paused;

		void				orderBoxesFromLeftToRight();
	};
}

#endif	// POWN_H__
