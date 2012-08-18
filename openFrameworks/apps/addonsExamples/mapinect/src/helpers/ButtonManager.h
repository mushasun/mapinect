#ifndef BUTTON_MANAGER_H__
#define BUTTON_MANAGER_H__

#include "IButtonManager.h"

#include <map>

namespace mapinect
{
	class ButtonManager: public IButtonManager
	{
	public:
		ButtonManager();

		//// IButtonManager implementation
		int						addButton(const IButtonPtr& btn);
		void					removeButton(int id);
		
		virtual void			setIdle(const ofColor& color, int id);
		virtual void			setIdle(ofImage* img, int id);
		virtual void			setPressed(const ofColor& color, int id);
		virtual void			setPressed(ofImage* img, int id);
		virtual vector<ofVec3f> getVertexs(int id);

		//// INotification implementation
		void					objectDetected(const IObjectPtr&);
		void					objectUpdated(const IObjectPtr&);
		void					objectLost(const IObjectPtr&);
		void					objectMoved(const IObjectPtr&, const DataMovement&);
		void					objectTouched(const IObjectPtr&, const DataTouch&);

		void					draw();
		void					fireButtonEvent(DataTouch touch);

	private:

		map<int, IButtonPtr>	buttons;
	};
}

#endif	// BUTTON_MANAGER_H__
