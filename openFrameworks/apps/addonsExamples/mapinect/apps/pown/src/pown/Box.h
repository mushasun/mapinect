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
		Box(const IObjectPtr& object);
		virtual ~Box();

		inline void		updateModelObject(const IObjectPtr& ob)		{ object = ob; }

		void			update(float elapsedTime);
		void			draw();

		bool			testHit(Bolt* bolt);
		void			absorbBolt(Bolt* bolt);

	private:
		IObjectPtr		object;
		ofColor			color;
		ofColor			boostColor;

	};
}

#endif	// BOX_H__
