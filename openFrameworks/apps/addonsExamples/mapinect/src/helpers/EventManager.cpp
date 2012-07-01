#include "EventManager.h"

namespace mapinect {

	EventManager*	EventManager::instance = NULL;
	ofxMutex		EventManager::mutexInstance;

	EventManager::EventManager() : eventsToFire() { }

	void EventManager::addEvent(const MapinectEvent& mapinectEvent)
	{
		EventManager::mutexInstance.lock();

		if (instance == NULL)
		{
			instance = new EventManager();
		}

		instance->eventsToFire.push_back(mapinectEvent);

		EventManager::mutexInstance.unlock();
	}

	void EventManager::fireEvents(IApplication* listener, PCM* pcm)
	{
		EventManager::mutexInstance.lock();

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
				case kMapinectEventTypeObjectUpdated:
					listener->objectUpdated(e->object);
					break;
				case kMapinectEventTypeObjectLost:
					listener->objectLost(e->object);
					break;
				case kMapinectEventTypeObjectMoved:
					listener->objectMoved(e->object, e->movement);
					break;
				case kMapinectEventTypeObjectTouched:
					listener->btnManager->fireButtonEvent(e->touchPoint);
					listener->objectTouched(e->object, e->touchPoint);
					pcm->objectTouched(e->object, e->touchPoint);
					break;
				}
			}
			instance->eventsToFire.clear();
			
		}

		EventManager::mutexInstance.unlock();
	}

}
