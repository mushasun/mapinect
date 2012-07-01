#ifndef I_BUTTON_H__
#define I_BUTTON_H__
#include "ofMain.h"
#include "DataTouch.h"


namespace mapinect {
	enum ButtonMode
	{
		PLAIN_COLOR = 0,
		TEXTURED
	};

	enum ButtonEvent
	{
		PRESSED = 0,
		RELEASED,
		NO_CHANGE
	};
	static int btnIds = 0;
	class IButton;
	typedef boost::shared_ptr<IButton> IButtonPtr;

	class IButton{
	public:
		virtual ButtonEvent updateTouchPoints(DataTouch touch)				= 0;
		virtual void draw()													= 0;
		virtual int getId()  const											= 0;

		/*virtual void SetIdleTexture(ofImage img)							{ }
		virtual void SetPressedTexture(ofImage img)							{ }
		virtual void SetIdleColor(ofColor color)							{ }
		virtual void SetPressedColor(ofColor color)							{ }
		virtual void SetMode(ButtonMode mode)								{ }*/

	};
}

#endif