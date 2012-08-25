#ifndef BUTTON_MANAGER_H__
#define BUTTON_MANAGER_H__

#include "IButtonManager.h"

#include "INotification.h"
#include <map>

namespace mapinect
{
	class ButtonManager: public IButtonManager, public INotification
	{
	public:
		ButtonManager();

		//// IButtonManager implementation
		int						addButton(const IButtonPtr& btn);
		void					removeButton(int id);
		const IButtonPtr&		getButton(int id);
		
		//// INotification implementation
		void					objectDetected(const IObjectPtr&);
		void					objectUpdated(const IObjectPtr&);
		void					objectLost(const IObjectPtr&);
		void					objectMoved(const IObjectPtr&, const DataMovement&);
		//void					objectTouched(const IObjectPtr&, const DataTouch&);

		void					objectTouchedPCM(const IObjectPtr&, const DataTouch&);
		void					draw();
		//void					fireButtonEvent(DataTouch touch);

	private:

		map<int, IButtonPtr>	buttons;
	};
}

#endif	// BUTTON_MANAGER_H__
