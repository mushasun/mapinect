#ifndef DRAGGABLE_BUTTON_H__
#define DRAGGABLE_BUTTON_H__

#include "SimpleButton.h"

namespace mapinect {
	class DraggableButton;
	typedef boost::shared_ptr<DraggableButton> DraggableButtonPtr;

	class DraggableButton : public SimpleButton
	{
		public:
			DraggableButton(Polygon3D polygon, ofColor idle, ofColor pressed);
			DraggableButton(Polygon3D polygon, ofImage* idle, ofImage* pressed);
			~DraggableButton(void);

			virtual ButtonEvent updateTouchPoints(const DataTouch& touch);	
		private:
			float lastScale;
	};
}
#endif