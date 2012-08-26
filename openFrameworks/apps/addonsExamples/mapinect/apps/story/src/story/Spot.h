#ifndef SPOT_H__
#define SPOT_H__

#include "ofImage.h"
#include "Polygon3D.h"

using namespace mapinect;

namespace story
{
	class Spot
	{
	public:
		Spot();
		virtual ~Spot();

		static void		setup();

		void			update(float elapsedTime);
		void			draw();

		inline void		setActive(bool isActive) { active = isActive; }
		inline void		setSize(float radius) { size = radius; }
		void			setPosition(const ofVec3f& position);
		inline bool		isActive() { return active; }
	private:
		ofVec3f			position;
		float			size;
		Polygon3D		area;
		float			rotation;
		bool			active;
		static ofImage*	texture;
	};
}

#endif	// BOX_H__
