#ifndef BRICK_H__
#define BRICK_H__

#include "NoteBeat.h"
#include "IPolygon.h"
#include "ofColor.h"
#include "Box.h"

#include <map>
#include <set>
#include <list>

using namespace mapinect;

namespace pown
{
	class Box;

	class Brick
	{
	public:
		Brick(const NoteBeat& noteBeat, const Polygon3D& hitPolygon, const vector<ofVec3f>& drawVertexs, const ofFloatColor& color);

		void					update(float elapsedTime);
		void					draw() const;

		inline const NoteBeat&	getNoteBeat() const					{ return noteBeat; }
		inline const Polygon3D&	getHitPolygon() const				{ return hitPolygon; }
		inline void				setColor(const ofColor& color)		{ this->color = color; }

	private:
		NoteBeat				noteBeat;
		Polygon3D				hitPolygon;
		vector<ofVec3f>			drawVertexs;
		ofFloatColor			color;
	};

	class Wave
	{
	public:
		Wave(const NoteBeat& noteBeat, const ofFloatColor& color, float intensity, float intensitySpeed, float radiusSpeed);

		void			update(float elapsedTime);

		set<NoteBeat>	getNoteBeats() const;

		bool			isAlive() const;
		ofFloatColor	getColor() const;

	private:
		NoteBeat		noteBeat;
		ofFloatColor	color;
		float			intensity;
		float			intensitySpeed;
		int				radius;
		float			radiusSpeed;
		float			radiusTimer;
	};

	class BrickManager
	{
	public:
		BrickManager(const IPolygonPtr& floor);

		void					update(float elapsedTime);
		void					draw() const;
		Brick*					getBrick(int note, int beat) const;

		void					doBeat(map<int, Box*>& boxes);

		void					updateBox(Box* box) const;

	private:
		vector<Brick*>			bricks;
		list<Wave*>				waves;
		vector<ofColor>			notesColors;
		int						beat;
	};

}

#endif	// BRICK_H__
