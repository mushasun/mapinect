#ifndef EVENT_MANAGER_H__
#define EVENT_MANAGER_H__

#include "IApplication.h"
#include "ofxMutex.h"
#include <vector>

namespace mapinect {

	typedef enum
	{
		kMapinectEventTypeObjectDetected = 0,
		kMapinectEventTypeObjectUpdated,
		kMapinectEventTypeObjectLost,
		kMapinectEventTypeObjectMoved,
		kMapinectEventTypeObjectTouched
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

		MapinectEventType		type;
		IObjectPtr				object;
		DataMovement			movement;
		DataTouch				touchPoint;
	};

	class EventManager
	{
	public:
		static void addEvent(const MapinectEvent& mapinectEvent);

		static void fireEvents(IApplication* listener);

	private:
		EventManager();

		static EventManager*			instance;
		static ofxMutex					mutexInstance;

		vector<MapinectEvent>			eventsToFire;

	};
}

#endif	// EVENT_MANAGER_H__
