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
			~DraggableButton(void);

			virtual ButtonEvent updateTouchPoints(DataTouch touch);					
	};
}
#endif