#ifndef FLOOR_H__
#define FLOOR_H__

#include "Box.h"

#include <list>

using namespace mapinect;

namespace pown
{
	class FloorBrick
	{
	public:
		FloorBrick(const Polygon3D& polygon, const ofColor& color);

		void				update(float elapsedTime);
		void				draw() const;

		inline void			setColor(const ofColor& color)		{ this->color = color; }

	private:
		Polygon3D			polygon;
		ofColor				color;
	};

	class Floor
	{
	public:
		Floor(const IObjectPtr& object, const ofColor& color);
		virtual ~Floor();

		void				update(float elapsedTime);
		void				draw() const;

		bool				testHit(Bolt* bolt) const;

	private:
		IPolygonPtr&		object;
		ofColor				baseColor;
		vector<FloorBrick>	bricks;
	};
}

#endif	// FLOOR_H__
