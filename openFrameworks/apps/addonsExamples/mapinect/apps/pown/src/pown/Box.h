#ifndef BOX_H__
#define BOX_H__

#include "IObject.h"
#include "Bolt.h"

using namespace mapinect;

namespace pown
{
	class Box
	{
	public:
		Box(const IObjectPtr& object, const ofColor& color);
		virtual ~Box();

		inline void					updateModelObject(const IObjectPtr& ob)	{ object = ob; }

		inline const IObjectPtr&	getObject()								{ return object; }
		inline ofVec3f				getCenter() const						{ return object->getCenter(); }

		void						update(float elapsedTime);
		void						draw() const;

		virtual bool				testHit(Bolt* bolt) const;
		virtual void				absorbBolt(Bolt* bolt);

	protected:
		IObjectPtr					object;
		ofColor						color;
		ofColor						boostColor;

	};
}

#endif	// BOX_H__
