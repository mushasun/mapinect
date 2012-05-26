#include "Model.h"

#include "EventManager.h"

namespace mapinect
{

	Model::Model() { }

	void Model::addObject(const ModelObjectPtr& object)
	{
		ofxScopedMutex osm(objectsMutex);
		objects.push_back(object);
		IObject* iObject = dynamic_cast<IObject*>(object.get());
		if (iObject != NULL)
		{
			EventManager::addEvent(MapinectEvent(kMapinectEventTypeObjectDetected, iObject));
		}
	}

	void Model::removeObject(const ModelObjectPtr& object)
	{
		ofxScopedMutex osm(objectsMutex);
		for (vector<ModelObjectPtr>::iterator it = objects.begin(); it != objects.end(); ++it)
		{
			if (it->get() == object.get())
			{
				objects.erase(it);
				break;
			}
		}
		IObject* iObject = dynamic_cast<IObject*>(object.get());
		if (iObject)
		{
			EventManager::addEvent(MapinectEvent(kMapinectEventTypeObjectLost, iObject));
		}
	}

	void Model::resetObjects()
	{
		ofxScopedMutex osm(objectsMutex);
		for (vector<ModelObjectPtr>::iterator it = objects.begin(); it != objects.end(); ++it)
		{
			IObject* iObject = dynamic_cast<IObject*>(it->get());
			if (iObject)
			{
				EventManager::addEvent(MapinectEvent(kMapinectEventTypeObjectLost, iObject));
			}
		}
		objects.clear();
	}

	void Model::setTable(const TablePtr& newTable)
	{
		ofxScopedMutex osm(tableMutex);
		table = newTable;
	}

	void Model::resetTable()
	{
		ofxScopedMutex osm(tableMutex);
		table.reset();
	}

}
