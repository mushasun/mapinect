#ifndef BASE_BUTTON_H__
#define BASE_BUTTON_H__

#include "IButton.h"

#include <map>
#include "ofImage.h"

namespace mapinect {

	class BaseButton;
	typedef boost::shared_ptr<BaseButton> BaseButtonPtr;

	class BaseButton : public IButton
	{
	public:
		BaseButton(const ofColor& idle, const ofColor& pressed);
		BaseButton(ofImage* idle, ofImage* pressed);

		virtual ButtonEvent	updateTouchPoints(const DataTouch& touch);
		inline int			getId() const						{ return id; }													
		inline void			setIdle(const ofColor& color)		{ idleColor = color; }
		inline void			setIdle(ofImage* img)				{ idleTexture = img; }
		inline void			setPressed(const ofColor& color)	{ pressedColor = color; }
		inline void			setPressed(ofImage* img)			{ pressedTexture = img; }
		inline void			setDrawMode(const ButtonDrawMode& m){ mode = m; }

		virtual void		draw() = 0;

		inline bool			isPressed() const					{ return contacts.size() > 0; } 

	protected:
		virtual bool		isInTouch(const DataTouch& touch) = 0;

		inline
		map<int, DataTouch>	getContacts()						{ return contacts; }
		
		int					leaderTouch;
		bool				leaderChanged;
		
		ButtonDrawMode		mode;
		ofColor				idleColor;
		ofColor				pressedColor;
		ofImage*			idleTexture;
		ofImage*			pressedTexture;

	private:
		void				init();

		map<int, DataTouch>	contacts;
		int					id;
	};
}

#endif //BASE_BUTTON_H__