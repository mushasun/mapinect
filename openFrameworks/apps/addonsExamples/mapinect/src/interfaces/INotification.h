#ifndef I_NOTIFICATION_H__
#define I_NOTIFICATION_H__

#include "ofEvents.h"

#include "DataMovement.h"
#include "DataTouch.h"
#include "IButton.h"

namespace mapinect {
	
	/// <summary>
	/// IObjectNotification.h
	/// 
	/// Provides an interface for objects events stuff.
	/// </summary>
	class INotification {
	public:
		virtual void objectDetected(const IObjectPtr&)						{ }
		virtual void objectUpdated(const IObjectPtr&)						{ }
		virtual void objectLost(const IObjectPtr&)							{ }
		virtual void objectMoved(const IObjectPtr&, const DataMovement&)	{ }
		virtual void objectTouched(const IObjectPtr&, const DataTouch&)		{ }
		virtual void buttonPressed(const IButtonPtr&, const DataTouch&)		{ }
		virtual void buttonReleased(const IButtonPtr&, const DataTouch&)	{ }
		virtual void pointTouched(const DataTouch&)							{ }
		virtual void armStoppedMoving()										{ }
		/*IButtonManager*	btnManager;*/
	};
}

#endif	// I_APPLICATION_H__