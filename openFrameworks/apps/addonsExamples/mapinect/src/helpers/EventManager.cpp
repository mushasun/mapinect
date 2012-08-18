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

	bool compareEvents(MapinectEvent t1, MapinectEvent t2)
	{
		return t1.type < t2.type;
	}

	void EventManager::fireEvents()
	{
		EventManager::mutexInstance.lock();

		if (instance != NULL)
		{
			instance->eventsToFire.sort(compareEvents);
			for (list<MapinectEvent>::iterator e = instance->eventsToFire.begin();
				e != instance->eventsToFire.end(); ++e)
			{
				
					
					for (vector<INotification*>::iterator l = instance->object_listeners.begin();
							l != instance->object_listeners.end(); ++l)
					{
						INotification* listener = *l;
						map<int,MapinectEventType>::iterator evnt;
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
							evnt = instance->eventsSended.find(e->touchPoint.getId());
							if( evnt == instance->eventsSended.end() ||
								evnt->second == kMapinectEventTypeObjectTouched)
								listener->objectTouched(e->object, e->touchPoint);
							break;
						case kMapinectEventTypeButtonPressed:
							listener->buttonPressed(e->button);
							instance->eventsSended[e->touchPoint.getId()] = e->type;
							break;
						case kMapinectEventTypeButtonReleased:
							listener->buttonReleased(e->button);
							instance->eventsSended[e->touchPoint.getId()] = e->type;
							break;
						}
					}
				
			}
			instance->eventsToFire.clear();
		}

		EventManager::mutexInstance.unlock();
	}

	void EventManager::suscribe(INotification* listener)
	{
		EventManager::mutexInstance.lock();

		if (instance == NULL)
		{
			instance = new EventManager();
		}

		instance->object_listeners.push_back(listener);

		EventManager::mutexInstance.unlock();
	}

}

