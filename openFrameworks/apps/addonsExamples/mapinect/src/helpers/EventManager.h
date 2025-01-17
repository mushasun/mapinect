#ifndef EVENT_MANAGER_H__
#define EVENT_MANAGER_H__

#include <vector>

#include "ofxMutex.h"
#include "INotification.h"

namespace mapinect {

	typedef enum
	{
		kMapinectEventTypeObjectDetected = 0,
		kMapinectEventTypeObjectUpdated,
		kMapinectEventTypeObjectLost,
		kMapinectEventTypeObjectMoved,
		kMapinectEventTypeObjectTouched,
		kMapinectEventTypePointTouched,
		kMapinectEventTypeButtonPressed,
		kMapinectEventTypeButtonReleased,
		kMapinectEventTypeArmStoppedMoving
	} MapinectEventType;

	struct MapinectEvent
	{
	public:
		MapinectEvent(const MapinectEventType& type, const IObjectPtr& object)
			: type(type), object(object) { }
		MapinectEvent(const MapinectEventType& type, const IObjectPtr& object, const DataMovement& movement)
			: type(type), object(object), movement(movement) { }
		MapinectEvent(const MapinectEventType& type, const IObjectPtr& object, const DataTouch& touchPoint)
			: type(type), object(object), touchPoint(touchPoint) { }
		MapinectEvent(const MapinectEventType& type, const IButtonPtr& button, const DataTouch& touchPoint)
			: type(type), button(button), touchPoint(touchPoint) { }
		MapinectEvent(const MapinectEventType& type, const DataTouch& touchPoint)
			: type(type), touchPoint(touchPoint) { }
		MapinectEvent(const MapinectEventType& type)
			: type(type) { }

		MapinectEventType		type;
		IObjectPtr				object;
		IButtonPtr				button;
		DataMovement			movement;
		DataTouch				touchPoint;
	};

	class EventManager
	{
	public:
		static void addEvent(const MapinectEvent& mapinectEvent);

		static void fireEvents();

		static void suscribe(INotification* listener);

	private:
		EventManager();

		static EventManager*			instance;
		static ofxMutex					mutexInstance;

		list<MapinectEvent>				eventsToFire;
		vector<INotification*>			objectListeners;

	};
}

#endif	// EVENT_MANAGER_H__
