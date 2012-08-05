#ifndef FLOOR_H__
#define FLOOR_H__

#include "Box.h"

#include <list>

using namespace mapinect;

namespace pown
{
	class FloorBumpEffect
	{
	public:
		FloorBumpEffect(const ofColor& color, float widthPercent, const Line3D& lineA, const Line3D& lineB);

		void				update(float elapsedTime);
		void				draw() const;

		Line3D				bar(float advance, float length) const;
		bool				isAlive() const;

	private:
		ofColor				color;
		float				widthPercent;
		float				lifetime;
		Line3D				lineA, lineB;
	};

	class Floor : public Box
	{
	public:
		Floor(const IObjectPtr& object, const ofColor& color);
		virtual ~Floor();

		void				update(float elapsedTime);
		void				draw() const;

		bool				testHit(Bolt* bolt) const;
		void				absorbBolt(Bolt* bolt);

		inline IPolygonPtr	getPolygon() const			{ return object->getPolygons()[0]; }

	private:
		list<FloorBumpEffect>	bumpEffects;
	};
}

#endif	// FLOOR_H__
