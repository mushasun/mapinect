#ifndef I_MODE_MANAGER_H__
#define I_MODE_MANAGER_H__

namespace mapinect
{
	class IModeManager
	{
	public:
		virtual void enableObjectTracking() = 0;
		virtual void disableObjectTracking() = 0;
		virtual void enableTouchTracking() = 0;
		virtual void disableTouchTracking() = 0;
		virtual bool objectTrackingEnabled() = 0;
		virtual bool touchTrackingEnabled() = 0;
	};
}

#endif