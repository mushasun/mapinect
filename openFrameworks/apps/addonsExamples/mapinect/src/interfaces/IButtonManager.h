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

		virtual void setIdle(const ofColor& color, int id) = 0;
		virtual void setIdle(ofImage* img, int id) = 0;
		virtual void setPressed(const ofColor& color, int id) = 0;
		virtual void setPressed(ofImage* img, int id) = 0;
		virtual vector<ofVec3f> getVertexs(int id) = 0;
	};
}

#endif