#ifndef LIGHT_H__
#define LIGHT_H__

#include "ofVec3f.h"
#include "ofColor.h"

namespace pown
{
	class Light
	{
	public:
		Light(const ofFloatColor& ambient, const ofFloatColor& diffuse, const ofVec3f& pos);

		inline const ofVec3f&	getPos() { return pos; }
		inline void				setPos(const ofVec3f& pos) { this->pos = pos; }

		ofFloatColor			getColor(const ofFloatColor& color, const ofVec3f& vertex, const ofVec3f& normal) const;

	private:
		ofFloatColor	ambient;
		ofFloatColor	diffuse;
		ofVec3f			pos;
	};
}

#endif	// LIGHT_H__
