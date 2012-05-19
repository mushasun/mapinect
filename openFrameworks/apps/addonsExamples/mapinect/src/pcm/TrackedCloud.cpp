#include "TrackedCloud.h"

#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation.h>

#include "AlignmentDetector.h"
#include "Constants.h"
#include "Model.h"
#include "Globals.h"
#include "objectTypesEnum.h"
#include "PCPolyhedron.h"
#include "PCHand.h"
#include "pointUtils.h"
#include "Table.h"
#include "utils.h"

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

	void TrackedCloud::addCounter(int diff) {
		counter += diff;
		//cout << "counter: " << counter;
		if (counter <= 0) {
			if (hasObject()) {
				gModel->objectsMutex.lock();
					for (vector<ModelObjectPtr>::iterator iter = gModel->objects.begin();
							iter != gModel->objects.end(); ++iter)
					{
						if ((*iter).get() == objectInModel.get())
						{
							gModel->objects.erase(iter, iter + 1);
							break;
						}
					}
					objectInModel.reset();
				gModel->objectsMutex.unlock();
			}
		}
		else if(counter >= mapinect::TIMES_TO_CREATE_OBJ && !hasObject()) {
			counter = mapinect::TIMES_TO_CREATE_OBJ + 2;
				
			
				//////////////Para identificar si es un objeto o una mano/////////////////
			ObjectType objType = getObjectType(cloud);

			switch(objType)
			{
				case HAND:
					objectInModel = PCModelObjectPtr(new PCHand(cloud, objId));
					cout << "HAND DETECTED" << endl;
					break;
				case BOX:
					objectInModel = PCModelObjectPtr(new PCPolyhedron(cloud, objId));
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

	void TrackedCloud::updateCloud(const PCPtr& cloud_cluster)
	{
		this->cloud = cloud_cluster;
	}

	void TrackedCloud::removeMatching(){
		 matchingCloud.reset();
		 nearest = numeric_limits<int>::max();
		 minPointDif = numeric_limits<int>::max();
	}
	
	//Compara si trackedCloud se corresponde con la TrackedCloud actual
	//En caso de tener una correspondencia (matchingCloud) previamente asociada, la devuelve en removedCloud
	//y setea removed a false
	bool TrackedCloud::matches(const TrackedCloudPtr& trackedCloud, TrackedCloudPtr& removedCloud, bool &removed)
	{
		removed = false;
		if(this->hasObject())
		{
			Eigen::Affine3f transformation;
			if(matchingTrackedObjects(trackedCloud, transformation))
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
			PCPtr difCloud (new PC());
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
				if(objectInModel.get() != NULL)
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
			ofVec3f vMax,vMin;
			findPointCloudBoundingBox(objectInModel->getCloud(), vMin, vMax);

			int density = CLOUD_RES - objectInModel->getLod();
			

			ofVec3f halo(0.005, 0.005, 0.005);
			halo /= (float)objectInModel->getLod();
			vMin -= halo;
			vMax += halo;

			PCPtr cloud = getCloud(density);

			Eigen::Vector4f eMax,eMin;
			eMax[0] = vMax.x;
			eMax[1] = vMax.y;
			eMax[2] = vMax.z;
			eMin[0] = vMin.x;
			eMin[1] = vMin.y;
			eMin[2] = vMin.z;

			//Necesito alguna forma de pasar de coordenadas de kinect a indices para obtener esta nube
			/*PCPtr cloud = getPartialCloudRealCoords(ofPoint(min(vMax.x,vMin.x),min(vMax.y,vMin.y)),
													ofPoint(max(vMax.x,vMin.x),max(vMax.y,vMin.y)),	
													density);*/

			vector<int> indices;

			pcl::getPointsInBox(*cloud,eMin,eMax,indices);
			PCPtr nuCloud (new PC());
			
			for(int i = 0; i < indices.size(); i++)
				nuCloud->push_back(cloud->at(indices.at(i)));

			PCPtr nuCloudFiltered (new PC());
			PCPtr nuCloudFilteredNoTable (new PC());

			PassThrough<PointXYZ> pass;
			pass.setInputCloud (nuCloud);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (0.001, 4.0);
			pass.filter (*nuCloudFiltered);
			
			saveCloudAsFile("preFiltroMesa.pcd",*nuCloudFiltered);

			//Quito los puntos que pertenecen a la mesa
			TablePtr table = gModel->table;
			ModelCoefficients tableCoef = table->getCoefficients();
			PointIndices::Ptr tableIdx = adjustPlane(tableCoef,nuCloud);

			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (nuCloudFiltered);
			extract.setIndices (tableIdx);
			extract.setNegative (true);
			extract.filter (*nuCloudFilteredNoTable);

			saveCloudAsFile("postFiltroMesa.pcd",*nuCloudFilteredNoTable);
			///Commented for debug
			/*gModel->objectsMutex.lock();
			objectInModel->updateCloud(nuCloudFilteredNoTable);
			gModel->objectsMutex.unlock();*/
			
			///Added for debug
			gModel->objectsMutex.lock();
			objectInModel->addToModel(nuCloudFilteredNoTable);
			gModel->objectsMutex.unlock();

		}
	}

	bool TrackedCloud::operator==(const TrackedCloudPtr& other) const {
		return other.get() == this;
	}

	bool TrackedCloud::matchingTrackedObjects(const TrackedCloudPtr& tracked_temp, Eigen::Affine3f &transformation)
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
		PCPtr cluster = tracked_temp->getTrackedCloud();
		//PCPtr obj_cloud (new PointCloud<PointXYZ>(objectInModel->getCloud()));
		PCPtr obj_cloud (new PC(*cloud));

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
		//ofVec3f clusterNormal = normalEstimation(cluster);
		//ofVec3f objNormal = normalEstimation(obj_cloud);

		////Hallo la rotación http://www.gamedev.net/topic/472246-rotation-matrix-between-two-vectors/
		//ofVec3f w (objNormal);
		//w = w.cross(clusterNormal);

		//float angle = asin(w.length());
		//w = w.normalize();
		//Eigen::Vector3f axis (w.x, w.y, w.z);
		//Eigen::Affine3f rotation;
		//rotation = Eigen::AngleAxis<float>(angle,axis);

		////PCPtr out (new PointCloud<PointXYZ>());
		//pcl::transformPointCloud(*obj_cloud,*obj_cloud,t);
		////////////////////////////////////////////////////////////////////////


		PCPtr out2 (new PC());
		pcl::transformPointCloud(*obj_cloud,*obj_cloud,traslation);


		//transformation = traslation*rotation;
		transformation = traslation;

		/*PCPtr out3 (new PointCloud<PointXYZ>());
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