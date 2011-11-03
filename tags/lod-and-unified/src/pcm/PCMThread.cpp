#include "PCMThread.h"

#include "ofMain.h"
#include "pointUtils.h"
#include "ofxVecUtils.h"
#include "ofGraphicsUtils.h"
#include "ofxXmlSettings.h"
#include "Timer.h"
#include "utils.h"

using namespace std;

#define DEFAULT_NAME		"test"
#define WAIT_TIME_MS		5
namespace mapinect {

	void PCMThread::setup() {
		//Cargo archivo de config.
		ofxXmlSettings XML;
		if(XML.loadFile("PCM_Config.xml")){

			OCTREE_RES = XML.getValue("PCMConfig:OCTREE_RES", 0.1);
			MIN_DIFF_TO_PROCESS = XML.getValue("PCMConfig:MIN_DIFF_TO_PROCESS", 40);
			QUAD_HALO = XML.getValue("PCMConfig:QUAD_HALO", 10);
			DIFF_THRESHOLD = XML.getValue("PCMConfig:DIFF_THRESHOLD", 20);
			RES_IN_OBJ = XML.getValue("PCMConfig:RES_IN_OBJ", 0.001);
			RES_IN_OBJ2 = XML.getValue("PCMConfig:RES_IN_OBJ2", 0.001);
			DIFF_IN_OBJ = XML.getValue("PCMConfig:DIFF_IN_OBJ", 20);
			TIMES_TO_CREATE_OBJ = XML.getValue("PCMConfig:TIMES_TO_CREATE_OBJ", 5);
			MIN_DIF_PERCENT = XML.getValue("PCMConfig:MIN_DIF_PERCENT", 0.1);
			TIMES_TO_STABILIZE = XML.getValue("PCMConfig:TIMES_TO_STABILIZE", 10);
			MAX_Z = XML.getValue("PCMConfig:MAX_Z", 40.0);
			NORMAL_ESTIMATION_PERCENT = XML.getValue("PCMConfig:NORMAL_ESTIMATION_PERCENT", 0.01);
			MAX_CLUSTER_SIZE = XML.getValue("PCMConfig:MAX_CLUSTER_SIZE", 10000);
			MIN_CLUSTER_SIZE = XML.getValue("PCMConfig:MIN_CLUSTER_SIZE", 100);
			MAX_CLUSTER_TOLERANCE = XML.getValue("PCMConfig:MAX_CLUSTER_TOLERANCE", 0.02);
			MAX_TABLE_CLUSTER_SIZE = XML.getValue("PCMConfig:MAX_TABLE_CLUSTER_SIZE", 2000);
			MIN_TABLE_CLUSTER_SIZE = XML.getValue("PCMConfig:MIN_TABLE_CLUSTER_SIZE", 1000);
			MAX_TABLE_CLUSTER_TOLERANCE = XML.getValue("PCMConfig:MAX_TABLE_CLUSTER_TOLERANCE", 0.05);
			CLOUD_RES = XML.getValue("PCMConfig:CLOUD_RES", 10);
			TRANSLATION_DISTANCE_TOLERANCE = XML.getValue("PCMConfig:TRANSLATION_DISTANCE_TOLERANCE", 0.01);
			MAX_OBJ_LOD = XML.getValue("PCMConfig:MAX_OBJ_LOD", 2);
			MAX_UNIFYING_DISTANCE = XML.getValue("PCMConfig:MAX_UNIFYING_DISTANCE", 0.01);

			ARDUINO_COM_PORT = XML.getValue("PCMConfig:ARDUINO_COM_PORT", "COM3");
		}

		// Initialize point cloud
		cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
			(new pcl::PointCloud<pcl::PointXYZ>()); 
		cloud->width    = KINECT_WIDTH;
		cloud->height   = KINECT_HEIGHT;
		cloud->is_dense = false;
		cloud->points.resize (CLOUD_POINTS);

		// Inicializo octree
		octree = new octree::OctreePointCloudChangeDetector<PointXYZ>(OCTREE_RES); //Valor de resolucion sacado del ejemplo

		timer = 0;
		baseCloudSetted = false;
		noDifferencesCount = 0;
		detectMode = false;
		objId = 0;
		startThread(true, false);

	}

	//--------------------------------------------------------------
	void PCMThread::threadedFunction() {
		while (isThreadRunning()) {
			if (lock()) {

				if(!baseCloudSetted && gKinect->isFrameNew()) {
					setInitialPointCloud();
					baseCloudSetted = true;
				}

				if(detectMode)
				{
					Timer t;
					t.start();
					processDiferencesClouds();
					t.end();
				}

				unlock();
				ofSleepMillis(WAIT_TIME_MS);
			}
		}
	}

	

	PointCloud<PointXYZ>::Ptr PCMThread::getPartialCloud(ofPoint min, ofPoint max){
		//Calcular tamaño de la nube
		PointCloud<PointXYZ>::Ptr partialColud (new pcl::PointCloud<pcl::PointXYZ>);
		partialColud->width    = max.x - min.x;
		partialColud->height   = max.y - min.y;
		partialColud->is_dense = false;
		partialColud->points.resize (partialColud->width*partialColud->height);

		//Recorrer el mapa de distancias obteniendo sólo los que estén dentro del rectángulo

		register int centerX = (cloud->width >> 1);
		int centerY = (cloud->height >> 1);
		float bad_point = std::numeric_limits<float>::quiet_NaN();

		register float* depth_map = gKinect->getDistancePixels();

		int step = 1;
		register int depth_idx = 0;
		int absV, absU, cloud_idx = 0;
		for(int v = -centerY; v < centerY; ++v) {
			for(register int u = -centerX; u < centerX; ++u, ++depth_idx) {
				absV = v + centerY;
				absU = u + centerX;
				if( absV > min.y && absV < max.y && absU > min.x && absU < max.x)
				{
					pcl::PointXYZ& pt = partialColud->points[cloud_idx];

					// Check for invalid measurements o puntos que estan fuera del nearthreshold/farthreshold
					// TODO: limitar que depth_map[depth_idx] caiga dentro de nearThreshold/farThreshold
					// Lo intente hacer pero parece que esos valores no corresponden con lo que lee el kinect
					if(/*depth_map[depth_idx] > 100 || */depth_map[depth_idx] == 0){
						pt.x = pt.y = pt.z = 0;
						//continue;
					}
					else{
						//ofxVec3f point = kinect->getWorldCoordinateFor(u,v);

						pt.z = depth_map[depth_idx];
						pt.x = u;
						pt.y = v;

					}
					cloud_idx++;
				}
			}
		}
		//pcl::io::savePCDFileASCII ("test_pacial_pcd.pcd", *partialColud);
		return partialColud;	
	}

	

	//--------------------------------------------------------------
	void PCMThread::setInitialPointCloud() {
		cloud = getCloud();
		currentDiffcloud = cloud;
	}

	PointCloud<PointXYZ>::Ptr PCMThread::getDifferenceIdx(bool &isDif, int noise_filter) {
		PointCloud<PointXYZ>::Ptr difCloud (new PointCloud<PointXYZ>());
		PointCloud<PointXYZ>::Ptr secondCloud = getCloud();
		isDif = false;
		int dif;

		/*pcl::PCDWriter writer;
		writer.write<pcl::PointXYZ> ("base.pcd", *currentDiffcloud, false);
		writer.write<pcl::PointXYZ> ("dif.pcd", *secondCloud, false);
		*/
		dif = getDifferencesCloud(cloud,secondCloud,difCloud,OCTREE_RES);
		if(dif  > MIN_DIFF_TO_PROCESS){
			isDif = true;
		}
		return difCloud;
	}


		bool countIsZero(const TrackedCloud &trackedCloud) {
			return ((TrackedCloud)trackedCloud).getCounter() == 0;
		}

		PointCloud<PointXYZ>::Ptr PCMThread::getTableCluster(){
			PCDWriter writer;
			PointCloud<PointXYZ>::Ptr cloud = getCloud();
			PointCloud<PointXYZ>::Ptr filteredCloud (new PointCloud<PointXYZ>());


			// Quito los puntos que sean 0
			PassThrough<PointXYZ> pass;
			pass.setInputCloud (cloud);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (0.001, 4.0);
			//pass.setFilterLimitsNegative (true);
			pass.filter (*filteredCloud);
			//writer.write<pcl::PointXYZ> ("filteredCloud.pcd", *filteredCloud, false);


			//Separo en clusters
			PointCloud<PointXYZ>::Ptr tableCluster (new pcl::PointCloud<pcl::PointXYZ>);

			pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
			tree->setInputCloud (filteredCloud);

			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
			ec.setClusterTolerance (MAX_TABLE_CLUSTER_TOLERANCE);
			ec.setMinClusterSize (MIN_TABLE_CLUSTER_SIZE);
			ec.setMaxClusterSize (MAX_TABLE_CLUSTER_SIZE);
			ec.setSearchMethod (tree);
			ec.setInputCloud(filteredCloud);
			ec.extract (cluster_indices);


			//Busco el cluster más cercano
			int count = 1;
			float min_dist = numeric_limits<float>::max();
			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
					cloud_cluster->points.push_back (filteredCloud->points[*pit]); //*
				//writer.write<pcl::PointXYZ> ("cluster" + ofToString(++count) + ".pcd", *cloud_cluster, false);

				Eigen::Vector4f clusterCentroid;
				compute3DCentroid(*cloud_cluster,clusterCentroid);

				if(clusterCentroid.norm() < min_dist)
				{
					min_dist = clusterCentroid.norm();
					tableCluster = cloud_cluster;
				}
			}

			
			//writer.write<pcl::PointXYZ> ("table.pcd", *tableCluster, false);

			//Quito el plano más grande
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
			pcl::SACSegmentation<pcl::PointXYZ> seg;
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			std::vector<ofxVec3f> vCloudHull;

			//Remover outliers
			PointCloud<PointXYZ>::Ptr cloudTemp (new PointCloud<PointXYZ>(*tableCluster));
			// Optional
			seg.setOptimizeCoefficients (true);
			// Mandatory
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setMaxIterations (50);
			seg.setDistanceThreshold (0.009); //original: 0.01

			// Create the filtering object
			int i = 0, nr_points = cloudTemp->points.size ();
			seg.setInputCloud (cloudTemp);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				PointCloud<PointXYZ>::Ptr empty (new PointCloud<PointXYZ>());
				return empty;
			}

			//FIX
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_temp_inliers (new pcl::PointCloud<pcl::PointXYZ>());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_temp_outliers (new pcl::PointCloud<pcl::PointXYZ>());

			// Create the filtering object
			extract.setInputCloud (cloudTemp);
			extract.setIndices (inliers);
			extract.setNegative (true);


			if(cloud_p->size() != cloudTemp->size())
				extract.filter (*cloud_filtered_temp_outliers);

			cloudTemp = cloud_filtered_temp_outliers;

			return cloudTemp;
		}

		void PCMThread::processDiferencesClouds() {
			bool dif;
			//nDetectedObjects = openCVService->contourFinder.blobs.size();
			PointCloud<PointXYZ>::Ptr filteredCloud = getTableCluster();

			if(filteredCloud->empty())
			{
				cout << "cluster vacio!" << endl;
				return;
			}

			//Subdivido la nube de diferencias en clusters
			// Creating the KdTree object for the search method of the extraction
			pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
			tree->setInputCloud (filteredCloud);

			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
			ec.setClusterTolerance (MAX_CLUSTER_TOLERANCE); 
			ec.setMinClusterSize (MIN_CLUSTER_SIZE);
			ec.setMaxClusterSize (MAX_CLUSTER_SIZE);
			ec.setSearchMethod (tree);
			ec.setInputCloud(filteredCloud);
			ec.extract (cluster_indices);

			/*nDetectedObjects = cluster_indices.size();
			int j = 0;
			*/

			//Actualizo las detecciones temporales
			for (list<TrackedCloud>::iterator iter = trackedClouds.begin(); iter != trackedClouds.end(); iter++) {
				iter->addCounter(-1);
			}
			trackedClouds.remove_if(countIsZero);

			list<TrackedCloud*> nuevosClouds;

			PCDWriter writer;
			//writer.write<pcl::PointXYZ> ("tableTop.pcd", *filteredCloud, false);

			//separo en clusters
			int debugCounter = 0;
			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
					cloud_cluster->points.push_back (filteredCloud->points[*pit]); //*


				//writer.write<pcl::PointXYZ> ("cluster" + ofToString(debugCounter) + ".pcd", *cloud_cluster, false);

				nuevosClouds.push_back(new TrackedCloud(cloud_cluster));
				debugCounter++;
			}

			//Itero en todas las nuves encontradas buscando el mejor ajuste con un objeto encontrado
			list<TrackedCloud*> aProcesarClouds;
			list<TrackedCloud*> aAgregarClouds;

			debugCounter = 0;
			int max_iter = 10;
			do{
				for (list<TrackedCloud*>::iterator iter = nuevosClouds.begin(); iter != nuevosClouds.end(); iter++) {
					/*PCDWriter writer;

					writer.write<pcl::PointXYZ> ("vector" + ofToString(debugCounter) + ".pcd", *iter->getCloud(), false);*/
					TrackedCloud *removedCloud;
					bool removed = false;
					//Busco el mejor ajuste
					bool fitted = findBestFit(*iter, removedCloud, removed);

					if(removed)										// El objeto que ajusto ya tenia una nube que ajustaba
						aProcesarClouds.push_back(removedCloud);	// Agrego la nube que ajustaba para volver a iterar
					if(!fitted)										// Si no encuentra objeto que ajuste, la agrega como una nueva nube
						aAgregarClouds.push_back(*iter);
					debugCounter++;
				}
				nuevosClouds = aProcesarClouds;
				aProcesarClouds.clear();
				max_iter--;
			}
			while (nuevosClouds.size() > 0 && max_iter > 0);

			////Actualizo los objetos con las nubes de mejor ajuste
			updateDetectedObjects();

			for (list<TrackedCloud*>::iterator iter = aAgregarClouds.begin(); iter != aAgregarClouds.end(); iter++) {
				trackedClouds.push_back(TrackedCloud(**iter));
			}
			////Los TrackedCloud* de aAgregarClouds quedan colgados?
			aAgregarClouds.clear();
		}

		//--------------------------------------------------------------
		void PCMThread::saveCloud(const string& name){
			cout << "saving: " << name << "..." << endl;

			register int centerX = (cloud->width >> 1);
			int centerY = (cloud->height >> 1);
			float bad_point = std::numeric_limits<float>::quiet_NaN();

			register float* depth_map = gKinect->getDistancePixels();
			float constant = 0.5f; //CAlculado segun lo que hay en la libreria de PCL
			int step = 1;
			register int depth_idx = 0;
			for(int v = -centerY; v < centerY; ++v) {
				for(register int u = -centerX; u < centerX; ++u, ++depth_idx) {
					pcl::PointXYZ& pt = cloud->points[depth_idx];
					// Check for invalid measurements
					if (depth_map[depth_idx] == 0)
					{
						// not valid
						pt.x = pt.y = pt.z = bad_point;
						continue;
					}
					pt.z = depth_map[depth_idx];
					pt.x = u * constant;
					pt.y = v * constant;
					//pt.x = u * pt.z * constant;
					//pt.y = v * pt.z * constant;
				}
			}
			//pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
			/*PointCloud<PointXYZ>::Ptr cloud_source_ptr; 

			cloud_source_ptr = cloud->makeShared(); */
			//viewer.showCloud (cloud);

			string filename = name + PCD_EXTENSION;
			pcl::io::savePCDFileASCII (filename, *cloud);
			std::cerr << "Saved " << cloud->points.size () << " data points to " << filename << std::endl;
		}

		void PCMThread::savePartialCloud(ofPoint min, ofPoint max, int id, const string& name){
			//Calcular tamaño de la nube
			cout << "saving: " << name << "..." << endl;
			PointCloud<PointXYZ>::Ptr partialColud = getPartialCloud(min,max);
			std::stringstream ss;
			ss << name << "-" << id << PCD_EXTENSION;
			pcl::io::savePCDFileASCII (ss.str(), *partialColud);
			std::cerr << "Saved " << partialColud->points.size () << " data points to " << ss.str() << std::endl;
		}

		PointCloud<PointXYZ>* PCMThread::loadCloud(const string& name) {
			cout << "loading: " << name << "..." << endl;
			pcl::PointCloud<pcl::PointXYZ>* tmpCloud = new pcl::PointCloud<PointXYZ>();
			string filename = name + PCD_EXTENSION;
			pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *tmpCloud);
			cout << name << " loaded!" << endl;
			return tmpCloud;
		}

		bool PCMThread::findBestFit(TrackedCloud* trackedCloud, TrackedCloud*& removedCloud, bool &removed) {
			for (list<TrackedCloud>::iterator iter = trackedClouds.begin(); iter != trackedClouds.end(); iter++) {
				if (iter->matches(trackedCloud, removedCloud, removed))
				{
					return true;
				}
			}
			return false;
		}

		void PCMThread::updateDetectedObjects() {
			for (list<TrackedCloud>::iterator iter = trackedClouds.begin(); iter != trackedClouds.end(); iter++) {
				if (iter->hasMatching())
				{
					iter->updateMatching();
					if (!(iter->hasObject())) {
						iter->addCounter(2);
					}
					else
						iter->addCounter(1);
					iter->removeMatching();
				}
			}
		}

		void PCMThread::reset()
		{
			ofxXmlSettings XML;
			if(XML.loadFile("PCM_Config.xml")){

				OCTREE_RES = XML.getValue("PCMConfig:OCTREE_RES", 0.1);
				MIN_DIFF_TO_PROCESS = XML.getValue("PCMConfig:MIN_DIFF_TO_PROCESS", 40);
				QUAD_HALO = XML.getValue("PCMConfig:QUAD_HALO", 10);
				DIFF_THRESHOLD = XML.getValue("PCMConfig:DIFF_THRESHOLD", 20);
				RES_IN_OBJ = XML.getValue("PCMConfig:RES_IN_OBJ", 0.001);
				RES_IN_OBJ2 = XML.getValue("PCMConfig:RES_IN_OBJ2", 0.001);
				DIFF_IN_OBJ = XML.getValue("PCMConfig:DIFF_IN_OBJ", 20);
				TIMES_TO_CREATE_OBJ = XML.getValue("PCMConfig:TIMES_TO_CREATE_OBJ", 5);
				MIN_DIF_PERCENT = XML.getValue("PCMConfig:MIN_DIF_PERCENT", 0.1);
				TIMES_TO_STABILIZE = XML.getValue("PCMConfig:TIMES_TO_STABILIZE", 10);
				MAX_Z = XML.getValue("PCMConfig:MAX_Z", 100);
				NORMAL_ESTIMATION_PERCENT = XML.getValue("PCMConfig:NORMAL_ESTIMATION_PERCENT", 0.01);
				MAX_CLUSTER_SIZE = XML.getValue("PCMConfig:MAX_CLUSTER_SIZE", 10000);
				MIN_CLUSTER_SIZE = XML.getValue("PCMConfig:MIN_CLUSTER_SIZE", 100);
				MAX_TABLE_CLUSTER_SIZE = XML.getValue("PCMConfig:MAX_TABLE_CLUSTER_SIZE", 2000);
				MIN_TABLE_CLUSTER_SIZE = XML.getValue("PCMConfig:MIN_TABLE_CLUSTER_SIZE", 1000);
				MAX_TABLE_CLUSTER_TOLERANCE = XML.getValue("PCMConfig:MAX_TABLE_CLUSTER_TOLERANCE", 0.05);
				MAX_CLUSTER_TOLERANCE = XML.getValue("PCMConfig:MAX_CLUSTER_TOLERANCE", 0.02);
				CLOUD_RES = XML.getValue("PCMConfig:CLOUD_RES", 10);
				TRANSLATION_DISTANCE_TOLERANCE = XML.getValue("PCMConfig:TRANSLATION_DISTANCE_TOLERANCE", 0.01);
				MAX_OBJ_LOD = XML.getValue("PCMConfig:MAX_OBJ_LOD", 2);
			}

			trackedClouds.clear();
		}
		

	//--------------------------------------------------------------
	ofxVec3f PCMThread::normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr plane, pcl::PointIndices::Ptr indicesptr){
		//Calculo de normales
		int sampleSize = indicesptr->indices.size();
		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud (plane);
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
		ne.setSearchMethod (tree);

		ne.setIndices(indicesptr);
		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch (0.3);

		// Compute the features
		ne.compute (*cloud_normals);

		ofxVec3f result(0,0,0);
		//Promedio las normales
		float bad_point = std::numeric_limits<float>::quiet_NaN();
		for(int i = 0; i < sampleSize ; i++){
			pcl::Normal normal = cloud_normals->at(i);
			if (normal.normal_z != bad_point && normal.normal_z == normal.normal_z)
				result += PCLNORMAL_OFXVEC3F(cloud_normals->at(i));
		}

		result /= sampleSize;
		result = result.normalize();
		return result;
	}

}
