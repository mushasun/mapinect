#ifndef BOX_H__
#define BOX_H__

#include "IObject.h"
#include "ofImage.h"

using namespace mapinect;

namespace story
{
	class Box
	{
	public:
		Box(const IObjectPtr& object);
		virtual ~Box();

		inline void		updateModelObject(const IObjectPtr& ob)		{ object = ob; }

		void			update(float elapsedTime);
		void			draw();

	private:
		IObjectPtr		object;
		ofImage*		texture;
		
	};
}

#endif	// BOX_H__
