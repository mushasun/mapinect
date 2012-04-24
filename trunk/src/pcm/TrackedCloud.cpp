#include "TrackedCloud.h"

#include "utils.h"
#include "objectTypesEnum.h"
#include "PCPolyhedron.h"
#include "PCHand.h"
#include "pointUtils.h"
#include <pcl/registration/transformation_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include "AlignmentDetector.h"

namespace mapinect {
	TrackedCloud::TrackedCloud(PointCloud<PointXYZ>::Ptr cloud) {
		this->cloud = cloud;
		counter = 2;
		objectInModel = NULL;
		matchingCloud = NULL;
		nearest = numeric_limits<int>::max();
		minPointDif = numeric_limits<int>::max();
		needApplyTransformation = false;
		needRecalculateFaces = false;
		hand = false;


		features_computed = false;
		search_method_xyz_ = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr();
		normal_radius_ = 0.02f;
		feature_radius_ = 0.02f;
	}

	TrackedCloud::TrackedCloud(PointCloud<PointXYZ>::Ptr cloud, bool isHand, bool forceCreate) {
		this->cloud = cloud;
		objectInModel = NULL;
		matchingCloud = NULL;
		nearest = numeric_limits<int>::max();
		minPointDif = numeric_limits<int>::max();
		needApplyTransformation = false;
		needRecalculateFaces = false;
		hand = isHand;

		if(isHand)
		{
			//Fuerzo crear el objeto
			counter = TIMES_TO_CREATE_OBJ;
			if(forceCreate)
				addCounter(0);
		}
		else
			counter = 2;
		
	}

	TrackedCloud::TrackedCloud() {
		objectInModel = NULL;
		matchingCloud = NULL;
		nearest = numeric_limits<int>::max();
		minPointDif = numeric_limits<int>::max();
		needApplyTransformation = false;
		needRecalculateFaces = false;
		hand = false;
	}

	TrackedCloud::~TrackedCloud() {

	}

	void TrackedCloud::addCounter(int diff) {
		counter += diff;
		//cout << "counter: " << counter;
		if (counter <= 0) {
			if (hasObject()) {
				gModel->objectsMutex.lock();
					gModel->objects.remove(objectInModel);
					delete objectInModel;
					objectInModel = NULL;
				gModel->objectsMutex.unlock();
			}
		}
		else if(counter >= TIMES_TO_CREATE_OBJ && !hasObject()) {
			counter = TIMES_TO_CREATE_OBJ + 2;
				
			
				//////////////Para identificar si es un objeto o una mano/////////////////
			ObjectType objType = getObjectType(cloud);

			switch(objType)
			{
				case HAND:
					objectInModel = new PCHand(cloud, cloud, objId);
					cout << "HAND DETECTED" << endl;
					break;
				case BOX:
					objectInModel = new PCPolyhedron(cloud, cloud, objId);
					cout << "BOX DETECTED" << endl;
					break;
				case UNRECOGNIZED:
					cout << "UNRECOGNIZED!" << endl;
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
				gModel->objectsMutex.lock();
					objId++;
					objectInModel->detectPrimitives();
					gModel->objects.push_back(objectInModel);
				gModel->objectsMutex.unlock();
			}
		}
	}

	void TrackedCloud::updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster)
	{
		this->cloud = cloud_cluster;
	}

	void TrackedCloud::removeMatching(){
		 delete matchingCloud;
		 matchingCloud = NULL;
		 nearest = numeric_limits<int>::max();
		 minPointDif = numeric_limits<int>::max();
	}
	
	//Compara si trackedCloud se corresponde con la TrackedCloud actual
	//En caso de tener una correspondencia (matchingCloud) previamente asociada, la devuelve en removedCloud
	//y setea removed a false
	bool TrackedCloud::matches(TrackedCloud* trackedCloud, TrackedCloud*& removedCloud, bool &removed)
	{
		removed = false;
		if(this->hasObject())
		{
			Eigen::Affine3f transformation;
			if(matchingTrackedObjects(*trackedCloud,transformation))
			{
				objectInModel->setTransformation(&transformation);
				
				if(matchingCloud != NULL)
				{
					removedCloud = matchingCloud;
					removed = true;
				}

				matchingCloud = trackedCloud;
				return true;
			}
		}
		else //Si no tiene un objeto asociado, sólo compara las distancias entre las distintas nubes.
		{
			PointCloud<PointXYZ>::Ptr difCloud (new PointCloud<PointXYZ>);
			Eigen::Vector4f clusterCentroid;
			Eigen::Vector4f objCentroid;
			compute3DCentroid(*trackedCloud->getTrackedCloud(),clusterCentroid);
			compute3DCentroid(*cloud,objCentroid);
			Eigen::Vector4f translationVector = clusterCentroid - objCentroid;
			if(translationVector.norm() < this->nearest)
			{
				this->nearest = translationVector.norm();
				if(matchingCloud != NULL)
				{
					removedCloud = matchingCloud;
					removed = true;
				}
				matchingCloud = trackedCloud;
				hand = trackedCloud->isPotentialHand();

				//getDifferencesCloud -> tiene problemas!!!
				//int dif = getDifferencesCloud(cloud, trackedCloud->getCloud(), difCloud, OCTREE_RES);
			
				//if(dif < DIFF_IN_OBJ)
				//{
				//	if(matchingCloud != NULL)
				//	{
				//		removedCloud = matchingCloud;
				//		removed = true;
				//	}
				//	//debug
				//	//matchingCloud = trackedCloud;
				//	return true;
				//}
				return true;
			}
			else
				return false;
		}
		return false;
	}

	void TrackedCloud::updateMatching()
	{
		if(needApplyTransformation || needRecalculateFaces || objectInModel == NULL)		//Necesito recalcular algo
		{ 
			if(hasMatching())
			{
				cloud = matchingCloud->getTrackedCloud();
				gModel->objectsMutex.lock();
				if(objectInModel != NULL)
				{
					/* Metodo viejo
					objectInModel->resetLod();
					objectInModel->setCloud(cloud);
					objectInModel->detectPrimitives();
					*/
				
					objectInModel->resetLod();
					objectInModel->addToModel(cloud);
					objectInModel->setCloud(cloud);
				}
				gModel->objectsMutex.unlock();
			}
			
		}
		else if(objectInModel != NULL && objectInModel->getLod() < MAX_OBJ_LOD)								//Si no llegue al nivel maximo de detalle, aumento el detalle
		{
			PCDWriter writer;
			ofxVec3f vMax,vMin;
			PointCloud<PointXYZ>::Ptr oldCloud (new PointCloud<PointXYZ>(objectInModel->getCloud()));
			findPointCloudBoundingBox(oldCloud, vMin, vMax);

			int density = CLOUD_RES - objectInModel->getLod();
			PointCloud<PointXYZ>::Ptr cloud = getCloud(density);
			//writer.write<pcl::PointXYZ>("nuObjCloud.pcd", *cloud, false);
			/*
			ofxVec3f sMin = gKinect->getScreenCoordsFromWorldCoords(vMin);
			ofxVec3f sMax = gKinect->getScreenCoordsFromWorldCoords(vMax);
			sMin -= ofxVec3f(density + 1, density + 1);
			sMax += ofxVec3f(density + 1, density + 1);

			PointCloud<PointXYZ>::Ptr cloud = getPartialCloudRealCoords(sMin, sMax, density);
			*/

			ofxVec3f halo(0.005, 0.005, 0.005);
			halo /= (float)objectInModel->getLod();
			vMin -= halo;
			vMax += halo;

			Eigen::Vector4f eMax,eMin;
			eMax[0] = vMax.x;
			eMax[1] = vMax.y;
			eMax[2] = vMax.z;
			eMin[0] = vMin.x;
			eMin[1] = vMin.y;
			eMin[2] = vMin.z;

			vector<int> indices;

			pcl::getPointsInBox(*cloud,eMin,eMax,indices);
			PointCloud<PointXYZ>::Ptr nuCloud (new PointCloud<PointXYZ>());
			
			for(int i = 0; i < indices.size(); i++)
				nuCloud->push_back(cloud->at(indices.at(i)));
			

			PointCloud<PointXYZ>::Ptr nuCloudFiltered (new PointCloud<PointXYZ>());
			PointCloud<PointXYZ>::Ptr nuCloudFilteredNoTable (new PointCloud<PointXYZ>());

			PassThrough<PointXYZ> pass;
			pass.setInputCloud (nuCloud);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (0.001, 4.0);
			//pass.setFilterLimitsNegative (true);
			pass.filter (*nuCloudFiltered);
			//writer.write<pcl::PointXYZ>("nuobj.pcd", *nuCloud, false);
			//writer.write<pcl::PointXYZ>("nuCloudFiltered.pcd", *nuCloudFiltered, false);

			//Quito los puntos que pertenecen a la mesa
			PCPolygon* table = getTable();
			ModelCoefficients tableCoef = table->getCoefficients();
			PointIndices::Ptr tableIdx = adjustPlane(tableCoef,nuCloud);

			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (nuCloudFiltered);
			extract.setIndices (tableIdx);
			extract.setNegative (true);
			extract.filter (*nuCloudFilteredNoTable);
			//writer.write<pcl::PointXYZ>("nuCloudFilteredNoTable.pcd", *nuCloudFilteredNoTable, false);

			gModel->objectsMutex.lock();
			objectInModel->updateCloud(nuCloudFilteredNoTable);
			gModel->objectsMutex.unlock();

		}
	}

	bool TrackedCloud::operator==(const TrackedCloud &other) const {
		return &other == this;
	}

	bool TrackedCloud::matchingTrackedObjects(TrackedCloud tracked_temp, Eigen::Affine3f &transformation)
	{
		///////////////////////////Metodo con Alignment////////////////////////
		//AlignmentDetector ad;
		//ad.setTargetCloud(tracked_temp);
		//ad.setTemplateCloud(*this);
		//AlignmentDetector::Result r = ad.align();

		//cout<<"result: " << r.fitness_score << endl;

		//if(r.fitness_score < nearest &&
		//	r.fitness_score < 0.00005f)
		//{
		//	cout << "match!" << endl;
		//	transformation = r.final_transformation;
		//	nearest = r.fitness_score;
		//	needApplyTransformation = true;
		//	needRecalculateFaces = true;

		//	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr f1 = this->getLocalFeatures();
		//	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr f2 = this->matchingCloud->getLocalFeatures();

		//	//pcl::io::savePCDFile("featureSource.pcd",*f1);
		//	//pcl::io::savePCDFile("featureTarget.pcd",*f2);

		//	//cout << transformation << endl;
		//	return true;
		//}

		///////////////////////////Fin Metodo con Alignment////////////////////////

		///////////////////////////Metodo Previo.//////////////////////////////
		//PCDWriter writer;
		//writer.write<pcl::PointXYZ> ("obj.pcd", *obj_cloud, false);
		/*writer.write<pcl::PointXYZ> ("cluster.pcd", *cluster, false);
		*/
		PointCloud<PointXYZ>::Ptr cluster = tracked_temp.getTrackedCloud();
		//PointCloud<PointXYZ>::Ptr obj_cloud (new PointCloud<PointXYZ>(objectInModel->getCloud()));
		PointCloud<PointXYZ>::Ptr obj_cloud (new PointCloud<PointXYZ>(*cloud));

		//Hallo la traslación 

		Eigen::Vector4f clusterCentroid;
		Eigen::Vector4f objCentroid;
		compute3DCentroid(*cluster,clusterCentroid);
		compute3DCentroid(*obj_cloud,objCentroid);

		Eigen::Vector4f translationVector = clusterCentroid - objCentroid;

		if(translationVector.norm() > nearest)
			return false;

		Eigen::Affine3f traslation;
		traslation = Eigen::Translation<float,3>(translationVector.x(),translationVector.y(),translationVector.z());

		//Elimino el calculo de las normales y rotacion/////////////////////////////////////////
		//ofxVec3f clusterNormal = normalEstimation(cluster);
		//ofxVec3f objNormal = normalEstimation(obj_cloud);

		////Hallo la rotación http://www.gamedev.net/topic/472246-rotation-matrix-between-two-vectors/
		//ofxVec3f w (objNormal);
		//w = w.cross(clusterNormal);

		//float angle = asin(w.length());
		//w = w.normalize();
		//Eigen::Vector3f axis (w.x, w.y, w.z);
		//Eigen::Affine3f rotation;
		//rotation = Eigen::AngleAxis<float>(angle,axis);

		////PointCloud<PointXYZ>::Ptr out (new PointCloud<PointXYZ>());
		//pcl::transformPointCloud(*obj_cloud,*obj_cloud,t);
		////////////////////////////////////////////////////////////////////////


		PointCloud<PointXYZ>::Ptr out2 (new PointCloud<PointXYZ>());
		pcl::transformPointCloud(*obj_cloud,*obj_cloud,traslation);


		//transformation = traslation*rotation;
		transformation = traslation;

		/*PointCloud<PointXYZ>::Ptr out3 (new PointCloud<PointXYZ>());
		pcl::transformPointCloud(*cluster,*out3,t2);*/


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
		nearest = translationVector.norm();
		if(nearest > TRANSLATION_DISTANCE_TOLERANCE)
			needApplyTransformation = true;
		else
			needApplyTransformation = false;

		return true;
		//////////////////////////////////////////////////////////
		///////////////////////////Fin Metodo Previo.//////////////////////////////
	}

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr TrackedCloud::getLocalFeatures ()
	{
		if(!features_computed)
		{
			processInput ();
		}
		return features_;
	}

	void TrackedCloud::processInput()
	{
		computeSurfaceNormals();
		computeLocalFeatures();
    }

	 // Compute the surface normals
    void TrackedCloud::computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (cloud);
	  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>());
      norm_est.setSearchMethod (tree);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void TrackedCloud::computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (cloud);
      fpfh_est.setInputNormals (normals_);
	  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>());
      fpfh_est.setSearchMethod (tree);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }
}