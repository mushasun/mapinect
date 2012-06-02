#include "Model.h"

#include "EventManager.h"
#include "PCModelObject.h"

namespace mapinect
{

	Model::Model() { }

	PCPtr Model::getCloudSum() const
	{
		PCPtr cloud(new PC);
		{
			ofxScopedMutex osm(objectsMutex);
			// TODO: se puede hacer cache para mejorar performance
			for (vector<ModelObjectPtr>::const_iterator ob = objects.begin(); ob != objects.end(); ++ob)
			{
				PCModelObject* pcm = dynamic_cast<PCModelObject*>(ob->get());
				if (pcm != NULL)
				{
					*cloud += *(pcm->getCloud());
				}
			}
		}
		{
			ofxScopedMutex osmt(tableMutex);
			if (table.get() != NULL)
			{
				*cloud += *(table->getCloud());
			}
		}
		return cloud;
	}

	vector<IObjectPtr> Model::getMathModelApproximation() const
	{
		vector<IObjectPtr> mathModel;
		{
			ofxScopedMutex osmo(objectsMutex);
		}
		{
			ofxScopedMutex osmt(tableMutex);
			if (table.get() != NULL)
			{
				mathModel.push_back(table->getMathModelApproximation());
			}
		}
		return mathModel;
	}

	void Model::addObject(const ModelObjectPtr& object)
	{
		ofxScopedMutex osm(objectsMutex);
		objects.push_back(object);
		PCModelObject* pcmo = dynamic_cast<PCModelObject*>(object.get());
		if (pcmo != NULL)
		{
			IObjectPtr iObject = pcmo->getMathModelApproximation();
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
		PCModelObject* pcmo = dynamic_cast<PCModelObject*>(object.get());
		if (pcmo != NULL)
		{
			IObjectPtr iObject = pcmo->getMathModelApproximation();
			EventManager::addEvent(MapinectEvent(kMapinectEventTypeObjectLost, iObject));
		}
	}

	void Model::resetObjects()
	{
		ofxScopedMutex osm(objectsMutex);
		for (vector<ModelObjectPtr>::iterator it = objects.begin(); it != objects.end(); ++it)
		{
			PCModelObject* pcmo = dynamic_cast<PCModelObject*>(it->get());
			if (pcmo != NULL)
			{
				IObjectPtr iObject = pcmo->getMathModelApproximation();
				EventManager::addEvent(MapinectEvent(kMapinectEventTypeObjectLost, iObject));
			}
		}
		objects.clear();
	}

	void Model::setTable(const TablePtr& newTable)
	{
		ofxScopedMutex osm(tableMutex);
		table = newTable;
		IObjectPtr iObject = table->getMathModelApproximation();
		EventManager::addEvent(MapinectEvent(kMapinectEventTypeObjectDetected, iObject));
	}

	void Model::resetTable()
	{
		ofxScopedMutex osm(tableMutex);
		IObjectPtr iObject = table->getMathModelApproximation();
		EventManager::addEvent(MapinectEvent(kMapinectEventTypeObjectLost, iObject));
		table.reset();
	}

}
