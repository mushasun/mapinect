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

namespace mapinect {

	void TrackedCloud::init()
	{
		objectInModel.reset();
		matchingCloud.reset();
		nearest = numeric_limits<int>::max();
		minPointDif = numeric_limits<int>::max();
		needApplyTransformation = false;
		needRecalculateFaces = false;
		hand = false;
	}

	TrackedCloud::TrackedCloud()
	{
		init();
	}

	TrackedCloud::TrackedCloud(const PCPtr& cloud)
	{
		init();
		this->cloud = cloud;
		counter = 2;

		features_computed = false;
		normal_radius = 0.02f;
		feature_radius = 0.02f;
	}

	TrackedCloud::TrackedCloud(const PCPtr& cloud, bool isHand, bool forceCreate)
	{
		init();
		this->cloud = cloud;
		hand = isHand;

		if(isHand)
		{
			//Fuerzo crear el objeto
			counter = mapinect::TIMES_TO_CREATE_OBJ;
			if(forceCreate)
				addCounter(0);
		}
		else
			counter = 2;
		
	}

	TrackedCloud::~TrackedCloud()
	{

	}

	void TrackedCloud::addCounter(int diff)
	{
		counter += diff;
		//cout << "counter: " << counter;
		if (counter <= 0)
		{
			if (hasObject())
			{
				gModel->removeObject(objectInModel);
				objectInModel.reset();
			}
		}
		else if(counter >= mapinect::TIMES_TO_CREATE_OBJ && !hasObject())
		{
			counter = mapinect::TIMES_TO_CREATE_OBJ + 2;
			
			//////////////Para identificar si es un objeto o una mano/////////////////
			ObjectType objType = getObjectType(cloud);

			switch(objType)
			{
				case BOX:
					objectInModel = PCModelObjectPtr(new PCBox(cloud));
					cout << "BOX DETECTED" << endl;
					break;
				case UNRECOGNIZED:
					//cout << "UNRECOGNIZED!" << endl;
					counter-= 2;
					return;
					break;
			}
				////////////////////////////////////////////////////////////
				
				////Si se descomenta lo anterior, comentar esto.
			 //   if(hand)
				//{
				//	objectInModel = new PCHand(cloud, cloud, -99);
				//	cout << "Hand detected!" << endl;
				//}
				//else
				//{
				//	objectInModel = new PCPolyhedron(cloud, cloud, objId);
				//	cout << "New object!" << endl;
				//}
			if(objType != UNRECOGNIZED)
			{
				objectInModel->detectPrimitives();
				gModel->addObject(objectInModel);
			}
		}
	}

	void TrackedCloud::updateCloud(const PCPtr& cloud_cluster)
	{
		this->cloud = cloud_cluster;
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
		PCPtr obj_cloud (new PC(*cloud));

		Eigen::Vector4f clusterCentroid;
		Eigen::Vector4f objCentroid;
		compute3DCentroid(*cluster,clusterCentroid);
		compute3DCentroid(*obj_cloud,objCentroid);

		Eigen::Vector4f translationVector = clusterCentroid - objCentroid;
		nearest = translationVector.norm();

		Eigen::Affine3f traslation;
		traslation = Eigen::Translation<float,3>(translationVector.x(),translationVector.y(),translationVector.z());

		pcl::transformPointCloud(*obj_cloud,*obj_cloud,traslation);
		matchingCloud = trackedCloud;

		if(hasObject())
		{
			///Elimino la comparación de diferencia de nubes, solo tomo la mas cercana.
			///Pero usa un limite de diferencia entre nubes para saber si recalcular las caras
			///////////////////////
			int difCount = getDifferencesCount(obj_cloud, cluster, RES_IN_OBJ2);;
			int maxDif = obj_cloud->points.size() * MIN_DIF_PERCENT;

			if(difCount > maxDif)
				needRecalculateFaces = true;
			else
				needRecalculateFaces = false;
			////////////////////////////////////////////////////////
			if(nearest > TRANSLATION_DISTANCE_TOLERANCE)
				needApplyTransformation = true;
			else
				needApplyTransformation = false;
		}
		
		return removed;
	}

	
	//float TrackedCloud::matches(const TrackedCloudPtr& trackedCloud)
	//{
	//	return matchingTrackedObjects(trackedCloud);
	//}

	void TrackedCloud::updateMatching()
	{
		bool sendUpdate = false;
		if(needApplyTransformation || needRecalculateFaces || objectInModel.get() == NULL)		//Necesito recalcular algo
		{ 
			if(hasMatching())
			{
				gModel->objectsMutex.lock();
				cloud = matchingCloud->getTrackedCloud();
				if(objectInModel.get() != NULL)
				{
					sendUpdate = true;
					objectInModel->resetLod();
					objectInModel->addToModel(cloud);
				}
				gModel->objectsMutex.unlock();
			}
			
		}
		else if(objectInModel.get() != NULL && objectInModel->getLod() < MAX_OBJ_LOD)								//Si no llegue al nivel maximo de detalle, aumento el detalle
		{
			sendUpdate = true;
			int density = CLOUD_RES - objectInModel->getLod();
			PCPtr cloud = getCloud(density);
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

			///Added for debug
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
	///Retorna -1 en caso de no matchear
	float TrackedCloud::matchingTrackedObjects(const TrackedCloudPtr& tracked_temp)
	{
		PCPtr diff;
		PCPtr cluster;
		PCPtr obj_cloud(new PC(*cloud));
		if(hasObject())
			cluster = getHalo(objectInModel->getvMin(),objectInModel->getvMax(),0.03,tracked_temp->getTrackedCloud());
		else
			cluster = tracked_temp->getTrackedCloud();

		if(cluster->size() < cloud->size() * 0.2)
			return -1;
		//Hallo la traslación 

		Eigen::Vector4f clusterCentroid;
		Eigen::Vector4f objCentroid;
		compute3DCentroid(*cluster,clusterCentroid);
		compute3DCentroid(*obj_cloud,objCentroid);

		Eigen::Vector4f translationVector = clusterCentroid - objCentroid;

		if(translationVector.norm() > nearest)
			return -1;
		else
			return translationVector.norm();
			
		
	}

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr TrackedCloud::getLocalFeatures ()
	{
		if(!features_computed)
		{
			processInput ();
		}
		return features;
	}

	void TrackedCloud::processInput()
	{
		computeSurfaceNormals();
		computeLocalFeatures();
    }

	 // Compute the surface normals
    void TrackedCloud::computeSurfaceNormals ()
    {
      normals = SurfaceNormalsPtr (new SurfaceNormals());

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (cloud);
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
      norm_est.setSearchMethod(tree);
      norm_est.setRadiusSearch(normal_radius);
      norm_est.compute(*normals);
    }

    // Compute the local feature descriptors
    void TrackedCloud::computeLocalFeatures ()
    {
		features = LocalFeaturesPtr(new LocalFeatures());

		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
		fpfh_est.setInputCloud (cloud);
		fpfh_est.setInputNormals (normals);
		SearchMethodPtr tree (new SearchMethod());
		fpfh_est.setSearchMethod (tree);
		fpfh_est.setRadiusSearch (feature_radius);
		fpfh_est.compute (*features);
    }
}