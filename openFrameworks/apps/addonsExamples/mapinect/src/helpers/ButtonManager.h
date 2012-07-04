#ifndef BUTTON_MANAGER_H__
#define BUTTON_MANAGER_H__

#include <map>
#include "DataTouch.h"
#include "IButtonManager.h"
#include "INotification.h"

namespace mapinect {

	class ButtonManager: public IButtonManager
	{
	public:
		//// IButtonManager implementation
		ButtonManager();
		void draw();
		int addButton(const IButtonPtr& btn);
		void removeButton(int id);
		void fireButtonEvent(DataTouch touch);

		//// INotification implementation
		void objectDetected(const IObjectPtr&);						
		void objectUpdated(const IObjectPtr&);					
		void objectLost(const IObjectPtr&);							
		void objectMoved(const IObjectPtr&, const DataMovement&);	
		void objectTouched(const IObjectPtr&, const DataTouch&);		
		void buttonPressed(const IButtonPtr&);						
		void buttonReleased(const IButtonPtr&);				
	private:

		map<int,IButtonPtr> buttons;
	};
}

#endif	// BUTTON_MANAGER_H__
