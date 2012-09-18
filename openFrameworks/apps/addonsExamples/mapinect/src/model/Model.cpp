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
			objectsMutex.lock();
			// TODO: se puede hacer cache para mejorar performance
			for (vector<ModelObjectPtr>::const_iterator ob = objects.begin(); ob != objects.end(); ++ob)
			{
				PCModelObject* pcm = dynamic_cast<PCModelObject*>(ob->get());
				if (pcm != NULL)
				{
					*cloud += *(pcm->getCloud());
				}
			}
			objectsMutex.unlock();
		}
		{
			tableMutex.lock();
			if (table.get() != NULL)
			{
				*cloud += *(table->getCloud());
			}
			tableMutex.unlock();

		}
		return cloud;
	}

	vector<IObjectPtr> Model::getMathModelApproximation() const
	{
		vector<IObjectPtr> mathModel;
		{
			objectsMutex.lock();
			for (vector<ModelObjectPtr>::const_iterator ob = objects.begin(); ob != objects.end(); ++ob)
			{
				PCModelObject* pcm = dynamic_cast<PCModelObject*>(ob->get());
				if (pcm != NULL)
				{
					mathModel.push_back(pcm->getMathModelApproximation());
				}
			}
			objectsMutex.unlock();
		}
		{
			tableMutex.lock();
			if (table.get() != NULL)
			{
				mathModel.push_back(table->getMathModelApproximation());
			}
			tableMutex.unlock();
		}
		return mathModel;
	}

	void Model::addObject(const ModelObjectPtr& object)
	{
		objectsMutex.lock();
		objects.push_back(object);
		PCModelObject* pcmo = dynamic_cast<PCModelObject*>(object.get());
		if (pcmo != NULL)
		{
			IObjectPtr iObject = pcmo->getMathModelApproximation();
			EventManager::addEvent(MapinectEvent(kMapinectEventTypeObjectDetected, iObject));
		}
		objectsMutex.unlock();
	}

	void Model::removeObject(const ModelObjectPtr& object)
	{
		objectsMutex.lock();
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
		objectsMutex.unlock();

	}

	void Model::resetObjects()
	{
		objectsMutex.lock();
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
		objectsMutex.unlock();
	}

	void Model::setTable(const TablePtr& newTable)
	{
		tableMutex.lock();
		bool updated = table.get() != NULL;
		table = newTable;
		IObjectPtr iObject = table->getMathModelApproximation();
		vector<ofVec3f> vertexs = table->getPolygonModelObject()->getMathModel().getVertexs();
		if (!updated) {
			EventManager::addEvent(MapinectEvent(kMapinectEventTypeObjectDetected, iObject));
			cout << "Table detected with vertexs:" << endl;
		} else {
			EventManager::addEvent(MapinectEvent(kMapinectEventTypeObjectUpdated, iObject));
			cout << "Table was updated, new vertexs:" << endl;
		}
		for(int i=0; i<vertexs.size();i++)
			cout << "	"<< i << "= (" << vertexs.at(i).x << "," << vertexs.at(i).y << "," << vertexs.at(i).z << ")" << endl;
		tableMutex.unlock();
	}

	void Model::resetTable()
	{
		tableMutex.lock();
		IObjectPtr iObject = table->getMathModelApproximation();
		EventManager::addEvent(MapinectEvent(kMapinectEventTypeObjectLost, iObject));
		table.reset();
		tableMutex.unlock();
	}

}
