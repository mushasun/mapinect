#ifndef SPOT_H__
#define SPOT_H__

#include "Box.h"

namespace pown
{
	class Spot
	{
	public:
		Spot(const ofVec3f& position);
		virtual ~Spot();

		void		update();
		void		draw();

		bool		testBox(Box* box);
		void		setBox(Box* box);

	private:
		ofVec3f		position;
		Box*		box;

	};
}

#endif	// BOX_H__
