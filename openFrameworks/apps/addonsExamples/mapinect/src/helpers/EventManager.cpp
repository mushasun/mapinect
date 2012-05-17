#include "EventManager.h"

namespace mapinect {

	EventManager*	EventManager::instance = NULL;
	ofxMutex		EventManager::mutexInstance;

	EventManager::EventManager() : eventsToFire() { }

	void EventManager::addEvent(const MapinectEvent& mapinectEvent)
	{
		ofxScopedMutex osm(EventManager::mutexInstance);

		if (instance == NULL)
		{
			instance = new EventManager();
		}

		instance->eventsToFire.push_back(mapinectEvent);
	}

	void EventManager::fireEvents(IApplication* listener)
	{
		ofxScopedMutex osm(EventManager::mutexInstance);

		if (instance != NULL)
		{
			for (vector<MapinectEvent>::iterator e = instance->eventsToFire.begin();
				e != instance->eventsToFire.end(); ++e)
			{
				switch (e->type)
				{
				case kMapinectEventTypeObjectDetected:
					listener->objectDetected(e->object);
					break;
				case kMapinectEventTypeObjectLost:
					listener->objectLost(e->object);
					break;
				case kMapinectEventTypeObjectMoved:
					listener->objectMoved(e->object, e->movement);
					break;
				case kMapinectEventTypeObjectTouched:
					listener->objectTouched(e->object, e->touchPoints);
					break;
				}
			}
			instance->eventsToFire.clear();
		}
	}

}
