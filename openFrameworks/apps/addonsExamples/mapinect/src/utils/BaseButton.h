#ifndef BASE_BUTTON_H__
#define BASE_BUTTON_H__

#include "IButton.h"
#include <set>

namespace mapinect {
	enum SButtonMode{
		kColor,
		kBgImg
	};

	class BaseButton;
	typedef boost::shared_ptr<BaseButton> BaseButtonPtr;

	class BaseButton : public IButton{
	public:
		BaseButton(ofColor idle, ofColor pressed);
		BaseButton(ofImage* idle, ofImage* pressed);
		virtual ButtonEvent updateTouchPoints(const DataTouch& touch);
		virtual void draw() = 0;							

		inline void setIdle(ofColor color)		{ idleColor = color; }
		inline void setIdle(ofImage* img)		{ texIdle = img; }
		inline void setPressed(ofColor color)	{ pressedColor = color; }
		inline void setPressed(ofImage* img)	{ texPressed = img; }

		inline int getId() const { return id; }													
		inline bool isPressed() { return contacts.size() > 0; } 
	protected:
		virtual bool isInTouch(const DataTouch& touch) = 0;
		inline map<int,DataTouch> getContacts() { return contacts; }
		int leaderTouch;
		bool leaderChanged;
		
		SButtonMode mode;
		ofColor pressedColor;
		ofColor idleColor;
		ofImage* texPressed;
		ofImage* texIdle;
	private:
		void init();

		map<int,DataTouch> contacts;
		int id;
	};
}

#endif //BASE_BUTTON_H__