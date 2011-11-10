#include "TrackedCloud.h"

#include "utils.h"
#include "PCPolyhedron.h"
#include "pointUtils.h"
#include <pcl/registration/transformation_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

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
	}

	TrackedCloud::TrackedCloud() {
		objectInModel = NULL;
		matchingCloud = NULL;
		nearest = numeric_limits<int>::max();
		minPointDif = numeric_limits<int>::max();
		needApplyTransformation = false;
		needRecalculateFaces = false;
	}

	TrackedCloud::~TrackedCloud() {

	}

	void TrackedCloud::addCounter(int diff) {
		counter += diff;
		if (counter <= 0) {
			if (hasObject()) {
				gModel->objectsMutex.lock();
					gModel->objects.remove(objectInModel);
					delete objectInModel;
					objectInModel = NULL;
				gModel->objectsMutex.unlock();
			}
		}
		else if(counter == TIMES_TO_CREATE_OBJ && !hasObject()) {
			counter = TIMES_TO_CREATE_OBJ + 2;
				
			gModel->objectsMutex.lock();
				objectInModel = new PCPolyhedron(cloud, cloud, objId);
				objId++;
				objectInModel->detectPrimitives();
				gModel->objects.push_back(objectInModel);
			gModel->objectsMutex.unlock();
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
	bool TrackedCloud::matches(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster) {
		/*if(this->hasObject())
		{
			Eigen::Affine3f transformation;
			PointCloud<PointXYZ>::Ptr obj_cloud (new PointCloud<PointXYZ>(objectInModel->getCloud()));
			if(matchingObjects(cloud_cluster,obj_cloud,transformation))
			{
				objectInModel->applyTransformation(&transformation);
				return true;
			}
		}
		else
		{
			PointCloud<PointXYZ>::Ptr difCloud (new PointCloud<PointXYZ>);
			int dif = getDifferencesCloud(cloud, cloud_cluster, difCloud, OCTREE_RES);
			if(dif < DIFF_IN_OBJ)
			{
				return true;
			}
		}*/
		return false;
	}

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
		else
		{
			PointCloud<PointXYZ>::Ptr difCloud (new PointCloud<PointXYZ>);
			Eigen::Vector4f clusterCentroid;
			Eigen::Vector4f objCentroid;
			compute3DCentroid(*trackedCloud->getTrackedCloud(),clusterCentroid);
			compute3DCentroid(*cloud,objCentroid);
			Eigen::Vector4f translationVector = clusterCentroid - objCentroid;
			if(translationVector.norm() > this->nearest)
				return false;
			this->nearest = translationVector.norm();
			if(matchingCloud != NULL)
			{
				removedCloud = matchingCloud;
				removed = true;
			}
			matchingCloud = trackedCloud;
			return true;
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
		}
		return false;
	}

	void TrackedCloud::updateMatching()
	{
		if(needApplyTransformation || needRecalculateFaces)		//Necesito recalcular algo
		{ 

			if(needApplyTransformation)
				cout << "applyTransform!" << endl;
			if(needRecalculateFaces)
				cout << "recalculate!" << endl;



			if(hasMatching())
				cloud = matchingCloud->getTrackedCloud();
			gModel->objectsMutex.lock();
			if(objectInModel != NULL)
			{
				objectInModel->resetLod();
				objectInModel->setCloud(cloud);
				/*if(needApplyTransformation)
					objectInModel->applyTransformation();*/
				//if(needRecalculateFaces)
					objectInModel->detectPrimitives();
			}
			gModel->objectsMutex.unlock();
			
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
			PassThrough<PointXYZ> pass;
			pass.setInputCloud (nuCloud);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (0.001, 4.0);
			//pass.setFilterLimitsNegative (true);
			pass.filter (*nuCloudFiltered);
			//writer.write<pcl::PointXYZ>("nuobj.pcd", *nuCloud, false);
			//writer.write<pcl::PointXYZ>("nuCloudFiltered.pcd", *nuCloudFiltered, false);

			gModel->objectsMutex.lock();
			objectInModel->updateCloud(nuCloudFiltered);
			gModel->objectsMutex.unlock();

		}
	}

	bool TrackedCloud::operator==(const TrackedCloud &other) const {
		return &other == this;
	}

	bool TrackedCloud::matchingTrackedObjects(TrackedCloud tracked_temp, Eigen::Affine3f &transformation)
	{
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
		////////////////////////////////////////////////////////
	}
}