#ifndef SPOT_H__
#define SPOT_H__

#include "Box.h"
#include "ofImage.h"

namespace pown
{
	class Spot
	{
	public:
		Spot(const ofVec3f& position);
		virtual ~Spot();

		static void		setup();

		void			update(float elapsedTime);
		void			draw();

		bool			testHit(Box* box);
		void			setBox(Box* box);

	private:
		ofVec3f			position;
		Box*			box;
		Polygon3D		area;
		float			rotation;

		static ofImage*	texture;
	};
}

#endif	// BOX_H__
