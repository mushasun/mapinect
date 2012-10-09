#include "TrackedCloud.h"

#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation.h>

#include "Constants.h"
#include "EventManager.h"
#include "Model.h"
#include "Globals.h"
#include "objectTypesEnum.h"
#include "PCPolyhedron.h"
#include "PCBox.h"
#include "pointUtils.h"
#include "Table.h"
#include "utils.h"
#include "EventManager.h"
#include "Frustum.h"

namespace mapinect {

	void TrackedCloud::init()
	{
		counter = 2;
		objectInModel.reset();
		matchingCloud.reset();
		nearest = numeric_limits<int>::max();
		minPointDif = numeric_limits<int>::max();
		needApplyTransformation = false;
		needRecalculateFaces = false;
		invalidCounter = 0;
		inViewField = true;
	}

	TrackedCloud::TrackedCloud()
	{
		init();
	}

	TrackedCloud::TrackedCloud(const PCPtr& cloud, bool floating)
	{
		init();
		this->cloud = cloud;
		this->floating = floating;
	}

	TrackedCloud::~TrackedCloud()
	{

	}

	void TrackedCloud::addCounter(int diff)
	{
		counter += diff;
		if (counter <= 0)
		{
			if (hasObject())
			{
				gModel->removeObject(objectInModel);
				objectInModel.reset();
			}
		}
		else if(counter >= Constants::OBJECT_FRAMES_TO_ACCEPT && !hasObject())
		{
			counter = Constants::OBJECT_FRAMES_TO_ACCEPT*2;
			
			//////////////Para identificar si es un objeto o una mano/////////////////
			ObjectType objType = getObjectType(cloud);

			switch(objType)
			{
				case BOX:
					objectInModel = PCModelObjectPtr(new PCBox(cloud));
					break;
				case UNRECOGNIZED:
					counter = 0;
					break;
			}

			if(objType != UNRECOGNIZED)
			{
				objectInModel->detectPrimitives();
				if(objectInModel->isValid())
					gModel->addObject(objectInModel);
				else
				{
					counter = 0;
					objectInModel.reset();
				}
			}
		}
	}

	void TrackedCloud::updateCloud(const PCPtr& cloud)
	{
		this->cloud = cloud;
	}

	void TrackedCloud::removeMatching()
	{
		 matchingCloud.reset();
		 nearest = numeric_limits<int>::max();
		 minPointDif = numeric_limits<int>::max();
	}
	
	bool TrackedCloud::confirmMatch(const TrackedCloudPtr& trackedCloud, TrackedCloudPtr& removedCloud)
	{
		bool removed = false;
		if(matchingCloud != NULL)
		{
			removedCloud = matchingCloud;
			removed = true;
		}

		PCPtr cluster;
		if(hasObject())
			cluster = getHalo(objectInModel->getvMin(),objectInModel->getvMax(),0.05,trackedCloud->getTrackedCloud());
		else
			cluster = trackedCloud->getTrackedCloud();
		PCPtr newObjectCloud(new PC(*cloud));

		ofVec3f clusterCentroid = computeCentroid(cluster);
		ofVec3f objCentroid = computeCentroid(newObjectCloud);
		ofVec3f translation = clusterCentroid - objCentroid;

		nearest = translation.length();

		Eigen::Affine3f translationMatrix;
		translationMatrix = Eigen::Translation<float,3>(translation.x, translation.y, translation.z);

		pcl::transformPointCloud(*newObjectCloud, *newObjectCloud, translationMatrix);
		matchingCloud = trackedCloud;

		if(hasObject())
		{
			///Elimino la comparación de diferencia de nubes, solo tomo la mas cercana.
			///Pero usa un limite de diferencia entre nubes para saber si recalcular las caras
			///////////////////////
			int difCount = getDifferencesCount(newObjectCloud, cluster, Constants::OBJECT_RECALCULATE_TOLERANCE());
			int maxDif = newObjectCloud->points.size() * Constants::OBJECT_CLOUD_DIFF_PERCENT;

			if(difCount > maxDif)
			{
		/*		cout << "dif: " << difCount << " --- max: " << maxDif << endl;  
				saveCloud("d1.pcd", *newObjectCloud);
				saveCloud("d2.pcd", *cluster);
*/
				needRecalculateFaces = true;
			}
			else
				needRecalculateFaces = false;
			
			if(nearest > Constants::OBJECT_TRANSLATION_TOLERANCE())
			{
				translationV = translation;
				needApplyTransformation = true;
			}
			else
			{
				needApplyTransformation = false;
				translationV = ofVec3f(0,0,0);
			}
		}
		
		return removed;
	}

	
	void TrackedCloud::updateMatching()
	{
		//Chequeo validez del objeto
		bool sendUpdate = false;
		
		gModel->objectsMutex.lock();
		if(objectInModel.get() != NULL &&
		   hasMatching())
		{
			//Elimino
			if(invalidCounter >= Constants::OBJECT_INVALID_FRAMES_TO_DELETE)
			{
				cout << "[     DELETED     ]" << endl;
				gModel->removeObject(objectInModel);
				objectInModel.reset();
			}
			////Recalculo
			else if(invalidCounter >= Constants::OBJECT_INVALID_FRAMES_TO_RESET)
			{
				cloud = matchingCloud->getTrackedCloud();
				objectInModel->resetLod();
				objectInModel->setCloud(cloud);
				// Si supera el limite, lo reseteo

				ObjectType objType = getObjectType(cloud);
				if(objType != UNRECOGNIZED)
				{
					cout << "[     RECALCULATING     ]" << endl;
					objectInModel->detectPrimitives();
					if(objectInModel->isValid())
					{
						PCPolyhedron* polyhedron = dynamic_cast<PCPolyhedron*>(objectInModel.get());
						if (polyhedron != NULL)
						{
							// Está dentro del frustum si todos sus vértices lo están
							if(Frustum::IsInFrustum(polyhedron->getVertexs()))
							{
								sendUpdate = true;
								invalidCounter = 0;
							}
						}
					}
				}
			}

			
			
		}
		gModel->objectsMutex.unlock();

		if(needApplyTransformation		|| 
		   needRecalculateFaces			|| 
		   objectInModel.get() == NULL	|| 
		   (invalidCounter > 0 && invalidCounter <= Constants::OBJECT_INVALID_FRAMES_TO_RESET))		//Necesito recalcular algo
		{ 
			if(hasMatching())
			{
				/*if(needApplyTransformation)
					cout << "transform" <<endl;
				if(needRecalculateFaces)
					cout << "faces" << endl;
				if(objectInModel.get() == NULL)
					cout << "null" << endl;
				if(invalidCounter)
					cout << "invalid count" << endl;
				cout << "recalculate" << endl;*/
				gModel->objectsMutex.lock();
				cloud = matchingCloud->getTrackedCloud();
				if(objectInModel.get() != NULL)
				{
					objectInModel->resetLod();
					objectInModel->addToModel(cloud);

					if(objectInModel->isValid())
					{
						invalidCounter = 0;
						sendUpdate = true;

						if(needApplyTransformation && objectInModel.get() != NULL)
						{
							DataMovement dm(translationV, ofVec3f());
							EventManager::addEvent(MapinectEvent(kMapinectEventTypeObjectMoved, 
							objectInModel->getMathModelApproximation(), dm));
							translationV = ofVec3f();
						}
					}
					else
						invalidCounter++;
				}
				gModel->objectsMutex.unlock();
			}
			
		}
		else if(objectInModel.get() != NULL && 
				objectInModel->getLod() < Constants::OBJECT_LOD_MAX && 
				objectInModel->isValid())
		//Si no llegue al nivel maximo de detalle, aumento
		{
			sendUpdate = true;
			int stride = Constants::CLOUD_STRIDE() - objectInModel->getLod();
			PCPtr cloud = getCloud(stride);
			PCPtr nuCloud = getHalo(objectInModel->getvMin(),objectInModel->getvMax(),0.005,cloud);

			//Quito los puntos que pertenecen a la mesa
			ModelCoefficients tableCoef;
			{
				gModel->tableMutex.lock();
				tableCoef = gModel->getTable()->getCoefficients();
				gModel->tableMutex.unlock();

			}
			PointIndices::Ptr tableIdx = adjustPlane(tableCoef, nuCloud);

			PCPtr nuCloudFilteredNoTable(new PC());
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (nuCloud);
			extract.setIndices (tableIdx);
			extract.setNegative (true);
			extract.filter (*nuCloudFilteredNoTable);
			saveCloud("postFiltroMesa.pcd",*nuCloudFilteredNoTable);

			{
				gModel->objectsMutex.lock();
				objectInModel->addToModel(nuCloudFilteredNoTable);
				gModel->objectsMutex.unlock();
			}
		}

		if (sendUpdate && objectInModel.get() != NULL)
		{
			EventManager::addEvent(MapinectEvent(kMapinectEventTypeObjectUpdated, objectInModel->getMathModelApproximation()));
		}
	}

	bool TrackedCloud::operator==(const TrackedCloudPtr& other) const {
		return other.get() == this;
	}

	///Retorna la distancia de matcheo entre las dos nubes
	///Retorna numeric_limits<float>::max() en caso de no matchear
	float TrackedCloud::matchingTrackedObjects(const TrackedCloudPtr& tracked) const
	{
		PCPtr diff;
		PCPtr cluster;
		
		//Da preferencia a nubes que está sobre la mesa
		if(matchingCloud != NULL)
		{
			if(!matchingCloud->isFloating() && tracked->isFloating())
			{
				return numeric_limits<float>::max();
			}
		}

		PCPtr newObjectCloud(new PC(*cloud));
		if(hasObject())
		{
			cluster = getHalo(objectInModel->getvMin(),objectInModel->getvMax(),0.03,tracked->getTrackedCloud());
		}
		else
		{
			cluster = tracked->getTrackedCloud();
		}
		saveCloud("newTrack.pcd", *newObjectCloud);
		saveCloud("prevTrack.pcd", *cluster);
		//if(cluster->size() < cloud->size() * 0.2)
		//	return -1;

		//Hallo la traslación 
		ofVec3f clusterCentroid = computeCentroid(cluster);
		ofVec3f objCentroid = computeCentroid(newObjectCloud);

		ofVec3f translation = clusterCentroid - objCentroid;

		float result = translation.length();

		//Da preferencia a los objetos apoyados en la mesa
		if(result > nearest)
		{
			if(matchingCloud != NULL &&
			   matchingCloud->isFloating() && !tracked->isFloating())
			{
				return result;
			}
			else
				return numeric_limits<float>::max();;
		}

		return result;
	}

}
