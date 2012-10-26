#ifndef BOX_H__
#define BOX_H__

#include "IObject.h"
#include "DataTouch.h"
#include "Brick.h"
#include "Light.h"
#include "ofImage.h"

using namespace mapinect;

namespace pown
{
	class Brick;

	class Box
	{
	public:
		Box(const IObjectPtr&, const ofFloatColor&, const NoteBeat&, int program);
		virtual ~Box();

		inline void					updateModelObject(const IObjectPtr& ob)	{ object = ob; }

		inline const IObjectPtr&	getObject()								{ return object; }
		inline ofVec3f				getCenter() const						{ return object->getCenter(); }
		inline const ofFloatColor&	getColor() const						{ return color; }
		inline const NoteBeat&		getNoteBeat() const						{ return noteBeat; }
		inline void					setNoteBeat(const NoteBeat& nb)			{ noteBeat = nb; }
		inline void					setProgram(int program)					{ this->program = program; }
		inline void					setLetterTexture(ofImage* texture)		{ this->letterTexture = texture; }
		inline void					setColorTexture(ofImage* texture)			{ this->colorTexture = texture; }

		void						update(float elapsedTime);
		void						draw(const Light& light) const;
		void						objectTouched(const IObjectPtr& object, const DataTouch& touchPoint);

		void						doBeat();

		bool						testHit(Brick* brick) const;

	protected:
		IObjectPtr					object;
		ofFloatColor				color;
		NoteBeat					noteBeat;

		ofFloatColor				boostColor;
		int							program;
		ofImage*					letterTexture;
		ofImage*					colorTexture;

	};
}

#endif	// BOX_H__
