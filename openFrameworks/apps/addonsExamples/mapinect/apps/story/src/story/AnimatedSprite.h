#ifndef ANIMATED_SPRITE_H__
#define ANIMATED_SPRITE_H__
#include "ofMain.h"
#include "Polygon3D.h"

using namespace mapinect;

namespace story
{
	class AnimatedSprite
	{
		public:
			AnimatedSprite(vector<ofImage*> images, float frameRate, Polygon3D pol);
			inline void setPolygon(Polygon3D pol);
			void update(float elapsed);
			void draw();

		private:
			void				polygonCorrection();
			float				frameRate;
			vector<ofImage*>	sprites;
			int					spritesCount;
			float				time;
			int					idx;
			Polygon3D			polygon;
	};

}
#endif	// ANIMATED_SPRITE_H__
