#ifndef PIANO_H__
#define PIANO_H__

#include "IApplication.h"

#include <set>
#include <map>

namespace piano
{
	class Piano : public IApplication
	{
	public:
		Piano();
		virtual ~Piano();

		virtual void		setup();
		virtual void		update(float elapsedTime);
		virtual void		draw();

		virtual void		objectTouched(const IObjectPtr&, const DataTouch&);
		virtual void		buttonPressed(const IButtonPtr&, const DataTouch&);
		virtual void		buttonReleased(const IButtonPtr&, const DataTouch&);
		virtual void		pointTouched(const DataTouch&);

	private:
		IPolygonPtr			floor;
		map<int, DataTouch>	touchPoints;
	};
}

#endif	// PIANO_H__
