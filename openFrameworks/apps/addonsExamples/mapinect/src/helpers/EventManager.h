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
		kMapinectEventTypeButtonPressed,
		kMapinectEventTypeButtonReleased
	} MapinectEventType;

	struct MapinectEvent
	{
	public:
		MapinectEvent(const MapinectEventType& type, const IObjectPtr& object)
			: type(type), object(object), movement(ofVec3f(), ofVec3f()) { }
		MapinectEvent(const MapinectEventType& type, const IObjectPtr& object, const DataMovement& movement)
			: type(type), object(object), movement(movement) { }
		MapinectEvent(const MapinectEventType& type, const IObjectPtr& object, const DataTouch& touchPoint)
			: type(type), object(object), movement(ofVec3f(), ofVec3f()), touchPoint(touchPoint) { }
		MapinectEvent(const MapinectEventType& type, const IButtonPtr& button)
			: type(type), button(button), movement(ofVec3f(), ofVec3f()) { }
		MapinectEvent(const MapinectEventType& type, const DataTouch& touchPoint)
			: type(type), movement(ofVec3f(), ofVec3f()), touchPoint(touchPoint) { }

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
		vector<INotification*>			object_listeners;

	};
}

#endif	// EVENT_MANAGER_H__
