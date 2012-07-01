#ifndef I_BUTTON_MANAGER_H__
#define I_BUTTON_MANAGER_H__

#include "IButton.h"
#include "DataTouch.h"
#include "IApplication.h"

namespace mapinect
{
	/*typedef enum
	{
	}*/
	class IApplication;

	class IButtonManager{
	public:
		//virtual void setup() = 0;
		virtual void draw() = 0;
		virtual int addButton(const IButtonPtr& btn) = 0;
		virtual void removeButton(int id) = 0;
		virtual void fireButtonEvent(DataTouch touch) = 0;
		virtual void setListener(IApplication* listener) = 0;
	
	};
}

#endif