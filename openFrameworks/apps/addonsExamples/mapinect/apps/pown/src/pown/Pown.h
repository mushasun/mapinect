#ifndef POWN_H__
#define POWN_H__

#include "IApplication.h"

#include <set>
#include <map>
#include "Box.h"
#include "Floor.h"
#include "Spot.h"

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
		virtual void		buttonPressed(const IButtonPtr&);
		virtual void		buttonReleased(const IButtonPtr&);

		void				testCollisions();
		void				handleBoltEmision(float elapsedTime);

	private:
		Floor*				floor;
		map<int, Box*>		boxes;
		set<Spot*>			spots;
		set<Bolt*>			bolts;

		int					emisor;
	};
}

#endif	// POWN_H__
