#ifndef DRAGGABLE_BUTTON_H__
#define DRAGGABLE_BUTTON_H__

#include "SimpleButton.h"

#include "Line3D.h"

namespace mapinect {
	class DraggableButton;
	typedef boost::shared_ptr<DraggableButton> DraggableButtonPtr;

	class DraggableButton : public SimpleButton
	{
		public:
			DraggableButton(Polygon3D polygon, ofColor idle, ofColor pressed);
			DraggableButton(Polygon3D polygon, ofImage* idle, ofImage* pressed);
			~DraggableButton(void);

			void	setRepeatBehavior(const vector<ofVec2f>& baseTexCoords, bool repeatS, bool repeatT);

			virtual ButtonEvent updateTouchPoints(const IObjectPtr& object, const DataTouch& touch);	
		private:
			bool				isTranslating;
			bool				isResizing;

			ofVec3f				translationBase;
			Line3D				resizeLineBase;
			Polygon3D			resizePolygonBase;
			
			enum ScalingDirection
			{
				kUniformScaling = 0,
				kHorizontalScaling,
				kVerticalScaling
			};
			ScalingDirection	resizeScalingDirection;

			vector<ofVec2f>		baseTexCoords;
			bool				repeatS;
			bool				repeatT;
	};
}
#endif