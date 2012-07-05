#ifndef SIMPLE_BUTTON_H__
#define SIMPLE_BUTTON_H__

#include "BaseButton.h"
#include <set>

namespace mapinect {
	class SimpleButton;
	typedef boost::shared_ptr<SimpleButton> SimpleButtonPtr;

	class SimpleButton : public BaseButton{
	public:
		SimpleButton(Polygon3D polygon, ofColor idle, ofColor pressed);
		SimpleButton(Polygon3D polygon, ofImage* idle, ofImage* pressed);
		virtual void draw();							
		virtual ButtonEvent updateTouchPoints(const DataTouch& touch);	
	protected:
		virtual bool isInTouch(const DataTouch& touch);
		Polygon3D polygon;
	private:
		void init();
	};
}

#endif //SIMPLE_BUTTON_H__