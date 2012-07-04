#ifndef SIMPLE_BUTTON_H__
#define SIMPLE_BUTTON_H__

#include "IButton.h"
#include <set>

namespace mapinect {
	enum SButtonMode{
		kColor,
		kBgImg
	};

	class SimpleButton;
	typedef boost::shared_ptr<SimpleButton> SimpleButtonPtr;

	class SimpleButton : public IButton{
	public:
		SimpleButton(Polygon3D polygon, ofColor idle, ofColor pressed);
		SimpleButton(Polygon3D polygon, ofImage* idle, ofImage* pressed);
		virtual ButtonEvent updateTouchPoints(DataTouch touch);
		virtual void draw();							
		inline int getId() const { return id; }											

		inline void setIdleColor(ofColor color)		{ idleColor = color; }
		inline void setPressedColor(ofColor color)	{ pressedColor = color; }
		inline bool isPressed() { return contacts.size() > 0; } 
	protected:
		inline map<int,DataTouch> getContacts() { return contacts; }
		Polygon3D polygon;
		int leaderTouch;
		bool leaderChanged;
	private:
		void init();
		
		SButtonMode mode;
		map<int,DataTouch> contacts;
		ofColor pressedColor;
		ofColor idleColor;
		ofColor currentColor;
		int id;

		ofImage* texPressed;
		ofImage* texIdle;
	};
}

#endif //SIMPLE_BUTTON_H__