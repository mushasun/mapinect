#ifndef I_BUTTON_MANAGER_H__
#define I_BUTTON_MANAGER_H__

#include "IButton.h"
#include "INotification.h"

namespace mapinect
{
	class IButtonManager: public INotification
	{
	public:
		virtual int addButton(const IButtonPtr& btn) = 0;
		virtual void removeButton(int id) = 0;
	};
}

#endif