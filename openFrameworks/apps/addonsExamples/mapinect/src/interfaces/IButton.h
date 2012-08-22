#ifndef I_BUTTON_H__
#define I_BUTTON_H__

#include "DataTouch.h"
#include "ofBaseTypes.h"

namespace mapinect
{
	enum ButtonDrawMode
	{
		kButtonDrawModePlain = 0,
		kButtonDrawModeTextured
	};

	enum ButtonEvent
	{
		kButtonEventPressed = 0,
		kButtonEventReleased,
		kButtonEventHolding,
		kButtonEventNotInButton
	};

	class IButton;
	typedef boost::shared_ptr<IButton> IButtonPtr;

	class IButton
	{
	public:
		virtual ButtonEvent updateTouchPoints(const DataTouch&)		= 0;
		virtual int getId() const									= 0;

		virtual void setIdle(ofImage*)								= 0;
		virtual void setIdle(const ofColor&)						= 0;
		virtual void setPressed(ofImage*)							= 0;
		virtual void setPressed(const ofColor&)						= 0;
		virtual void setDrawMode(const ButtonDrawMode&)				= 0;
		virtual vector<ofVec3f>		getVertexs()					= 0;

	};
}

#endif