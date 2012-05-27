#include "PCMThread.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>

#include "ofxXmlSettings.h"

#include "Constants.h"
#include "Globals.h"
#include "pointUtils.h"
#include "utils.h"

using namespace std;

#define WAIT_TIME_MS		20
#define PCM_CONFIG			"PCMConfig:"

namespace mapinect {
	PCMThread::PCMThread()
	{
		detectMode = false;
		isNewFrameAvailable = false;
		isNewForcedFrameAvailable = false;
	}

	void PCMThread::reset()
	{
		ofxXmlSettings XML;
		if(XML.loadFile("PCM_Config.xml")){

			OCTREE_RES = XML.getValue(PCM_CONFIG "OCTREE_RES", 0.1);
			MIN_DIFF_TO_PROCESS = XML.getValue(PCM_CONFIG "PCMConfig:MIN_DIFF_TO_PROCESS", 40);
			QUAD_HALO = XML.getValue(PCM_CONFIG "QUAD_HALO", 10);
			DIFF_THRESHOLD = XML.getValue(PCM_CONFIG "DIFF_THRESHOLD", 20);
			RES_IN_OBJ = XML.getValue(PCM_CONFIG "RES_IN_OBJ", 0.001);
			RES_IN_OBJ2 = XML.getValue(PCM_CONFIG "RES_IN_OBJ2", 0.001);
			DIFF_IN_OBJ = XML.getValue(PCM_CONFIG "DIFF_IN_OBJ", 20);
			TIMES_TO_CREATE_OBJ = XML.getValue(PCM_CONFIG "TIMES_TO_CREATE_OBJ", 5);
			MIN_DIF_PERCENT = XML.getValue(PCM_CONFIG "MIN_DIF_PERCENT", 0.1);
			TIMES_TO_STABILIZE = XML.getValue(PCM_CONFIG "TIMES_TO_STABILIZE", 10);
			MAX_Z = XML.getValue(PCM_CONFIG "MAX_Z", 40.0);
			NORMAL_ESTIMATION_PERCENT = XML.getValue(PCM_CONFIG "NORMAL_ESTIMATION_PERCENT", 0.01);
			MAX_CLUSTER_SIZE = XML.getValue(PCM_CONFIG "MAX_CLUSTER_SIZE", 10000);
			MIN_CLUSTER_SIZE = XML.getValue(PCM_CONFIG "MIN_CLUSTER_SIZE", 100);
			MAX_CLUSTER_TOLERANCE = XML.getValue(PCM_CONFIG "MAX_CLUSTER_TOLERANCE", 0.02);
			MAX_TABLE_CLUSTER_SIZE = XML.getValue(PCM_CONFIG "MAX_TABLE_CLUSTER_SIZE", 2000);
			MIN_TABLE_CLUSTER_SIZE = XML.getValue(PCM_CONFIG "MIN_TABLE_CLUSTER_SIZE", 1000);
			MAX_TABLE_CLUSTER_TOLERANCE = XML.getValue(PCM_CONFIG "MAX_TABLE_CLUSTER_TOLERANCE", 0.05);
			CLOUD_RES = XML.getValue(PCM_CONFIG "CLOUD_RES", 10);
			TRANSLATION_DISTANCE_TOLERANCE = XML.getValue(PCM_CONFIG "TRANSLATION_DISTANCE_TOLERANCE", 0.01);
			MAX_OBJ_LOD = XML.getValue(PCM_CONFIG "MAX_OBJ_LOD", 2);
			MAX_UNIFYING_DISTANCE = XML.getValue(PCM_CONFIG "MAX_UNIFYING_DISTANCE", 0.01);

			KINECT_WIDTH = XML.getValue(PCM_CONFIG "KINECT_WIDTH", KINECT_DEFAULT_WIDTH);
			KINECT_HEIGHT = XML.getValue(PCM_CONFIG "KINECT_HEIGHT", KINECT_DEFAULT_HEIGHT);
			KINECT_WIDTH_OFFSET = XML.getValue(PCM_CONFIG "KINECT_WIDTH_OFFSET", 0);
			KINECT_HEIGHT_OFFSET = XML.getValue(PCM_CONFIG "KINECT_HEIGHT_OFFSET", 0);
			MAX_UNIFYING_DISTANCE_PROJECTION = XML.getValue(PCM_CONFIG "MAX_UNIFYING_DISTANCE_PROJECTION", 0.01);
			TOUCH_DISTANCE = XML.getValue(PCM_CONFIG "TOUCH_DISTANCE", 0.02);
			HAND_SIZE = XML.getValue(PCM_CONFIG "HAND_SIZE", 0.20);

			MIN_ANGLES_FINGERS = parseArray(XML.getValue(PCM_CONFIG "MIN_ANGLES_FINGERS", "51.5,22.1,21.6,26.7"));
			MAX_ANGLES_FINGERS = parseArray(XML.getValue(PCM_CONFIG "MAX_ANGLES_FINGERS", "55,31,23.3,29.2"));
			MIN_LENGTH_FINGERS = parseArray(XML.getValue(PCM_CONFIG "MIN_LENGTH_FINGERS", ".079,.097,.099,.103,.086"));
			MAX_LENGTH_FINGERS = parseArray(XML.getValue(PCM_CONFIG "MAX_LENGTH_FINGERS", ".089,.106,.109,.109,.95"));
		}
	}

	void PCMThread::setup() {
		reset();

		tableClusterLastCentroid = ofVec3f(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);

		objectsThread.setup();

		// Start the cloud processing thread
		startThread(true, false);
	}

	//--------------------------------------------------------------
	void PCMThread::exit() {
		stopThread();
	}

	//--------------------------------------------------------------
	void PCMThread::newFrameAvailable()
	{
		ofxScopedMutex osm(isNewFrameAvailableMutex);
		isNewFrameAvailable = detectMode;
	}

	//--------------------------------------------------------------
	void PCMThread::newForcedFrameAvailable()
	{
		ofxScopedMutex osm(isNewFrameAvailableMutex);
		isNewForcedFrameAvailable = true;
	}

	//--------------------------------------------------------------
	void PCMThread::threadedFunction() {
		while (isThreadRunning()) {
			if (lock()) {
				
				bool newFrameAvailable = false;
				{
					ofxScopedMutex osm(isNewFrameAvailableMutex);
					newFrameAvailable = isNewFrameAvailable || isNewForcedFrameAvailable;
					isNewForcedFrameAvailable = false;
				}

				if(newFrameAvailable)
				{
					cout << "Procesando..." << endl;
					processCloud();
					cout << "Fin procesamiento" << endl;

				}
				
				unlock();
				ofSleepMillis(WAIT_TIME_MS);
			}
		}
	}
	
	//--------------------------------------------------------------
	PCPtr PCMThread::getObjectsOnTableTopCloud(){
		PCPtr cloud = getCloud();
		//saveCloudAsFile("rawCloud.pcd", *cloud);
		
		if (cloud->size() == 0) {
			return cloud;
		}

		//Separo en clusters
		PCPtr tableCluster (new PC());

		std::vector<pcl::PointIndices> cluster_indices =
			findClusters(cloud, MAX_TABLE_CLUSTER_TOLERANCE, MIN_TABLE_CLUSTER_SIZE, MAX_TABLE_CLUSTER_SIZE);

		//Busco el cluster más cercano
		int count = 1;
		float min_dist2 = MAX_FLOAT;
		ofVec3f newCentroid(tableClusterLastCentroid);
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			PCPtr cloud_cluster = getCloudFromIndices(cloud, *it);
			//savePC(*cloud_cluster, "cluster" + ofToString(count++) + ".pcd");

			Eigen::Vector4f clusterCentroid;
			compute3DCentroid(*cloud_cluster,clusterCentroid);
			ofVec3f ptoCentroid = ofVec3f(clusterCentroid.x(),clusterCentroid.y(),clusterCentroid.z());
			
			if (tableClusterLastCentroid.x == MAX_FLOAT)
			{
				if (ptoCentroid.squareLength() < newCentroid.squareLength())
				{
					newCentroid = ptoCentroid;
					tableCluster = cloud_cluster;
				}
			}
			else
			{
				if((ptoCentroid - tableClusterLastCentroid).squareLength() < min_dist2)
				{
					min_dist2 = (ptoCentroid - tableClusterLastCentroid).squareLength();
					tableCluster = cloud_cluster;
				}
			}
		}
		tableClusterLastCentroid = newCentroid;
			
		saveCloudAsFile("tableCluster.pcd", *tableCluster);

		//Quito el plano más grande
		pcl::ModelCoefficients coefficients;
		PCPtr remainingCloud = PCPtr(new PC());

		{
			ofxScopedMutex osm(gModel->tableMutex);
			if (gModel->getTable().get() == NULL)
			{
				PCPtr biggestPlaneCloud = extractBiggestPlane(tableCluster, coefficients, remainingCloud, 0.009);
				saveCloudAsFile("table.pcd", *biggestPlaneCloud);
				TablePtr table = TablePtr(new Table(coefficients, biggestPlaneCloud));
				table->detect();
				table->setDrawPointCloud(false);
				gModel->setTable(table);
			}
			else
			{
				//TODO: ACTUALIZAR LA NUBE DE LA MESA
				coefficients = gModel->getTable()->getCoefficients();
				for(int i = 0; i < tableCluster->size(); i++)
				{
					if(evaluatePoint(coefficients, POINTXYZ_OFXVEC3F(tableCluster->at(i))) > OCTREE_RES)
						remainingCloud->push_back(tableCluster->at(i));
				}
			}
		}
		// TODO: OJO A LA CONCURRENCIA SOBRE gModel!!
		/* Actualización de la mesa
		else if((tableClusterLastCentroid - tableClusterNewCentroid).norm() > 0.02)
		{
			cout << "table updated! -- " << min_dist <<endl;
			gModel->objectsMutex.lock();
			table->setCloud(cloud_filtered_temp_inliers);
			//table = new PCPolyhedron(cloud_filtered_temp_inliers, cloud_filtered_temp_inliers, -1);
			table->detectPrimitives();
			//table->setDrawPointCloud(false);
			gModel->table = table;
			tableClusterLastCentroid = tableClusterNewCentroid;
		}
		*/

		return remainingCloud;
	}

	PCPtr PCMThread::getDifferenceCloudFromModel(const PCPtr& cloud, vector<Polygon3D>& mathModel)
	{
		//saveCloudAsFile("sourceCloud.pcd", *cloud);

		PCPtr modelsCloud(new PC);
		{
			ofxScopedMutex osmo(gModel->objectsMutex);
			// TODO: se puede hacer cache para mejorar performance
			for(int i = 0; i < gModel->getObjects().size(); i++)
			{
				PCModelObject* pcm = dynamic_cast<PCModelObject*>(gModel->getObjects().at(i).get());
				if (pcm != NULL)
				{
					*modelsCloud += *(pcm->getCloud());
					vector<Polygon3D> pcmp(pcm->getMathModelApproximation());
					mathModel.insert(mathModel.begin(), pcmp.begin(), pcmp.end());
				}
			}

			ofxScopedMutex osmt(gModel->tableMutex);
			vector<Polygon3D> tp(gModel->getTable()->getMathModelApproximation());
			mathModel.insert(mathModel.begin(), tp.begin(), tp.end());
		}
		//saveCloudAsFile("modelsCloud.pcd", *modelsCloud);

		PCPtr filteredCloud(new PointCloud<PointXYZ>);
		int dif = getDifferencesCloud(modelsCloud, cloud, filteredCloud, 0.01);
		//cout << "Diferencia: " << ofToString(dif) << endl;

		//saveCloudAsFile("dif.pcd", *filteredCloud);

		return filteredCloud;
	}

	void PCMThread::processCloud()
	{
		bool dif;
		PCPtr objectsOnTableTopCloud = getObjectsOnTableTopCloud();

		objectsThread.setCloud(objectsOnTableTopCloud);

		// split the new cloud from the existing one
		vector<Polygon3D> mathModel;
		PCPtr differenceCloud = getDifferenceCloudFromModel(objectsOnTableTopCloud, mathModel);
		
		// touch detection and tracking

		vector<pcl::PointIndices> clusterIndices = findClusters(differenceCloud, MAX_CLUSTER_TOLERANCE, 40, 5000);
		
		map<Polygon3D*, vector<ofVec3f> >	touchPoints;

		for (int i = 0; i < clusterIndices.size(); ++i)
		{
			PCPtr cluster = getCloudFromIndices(differenceCloud, clusterIndices.at(i));
			saveCloudAsFile("cluster" + ofToString(i) + ".pcd", *cluster);

			// filter the points that are close to the math model
			vector<ofVec3f> vCluster(pointCloudToOfVecVector(cluster));
			for (vector<ofVec3f>::iterator v = vCluster.begin(); v != vCluster.end(); ++v)
			{
				for (vector<Polygon3D>::iterator p = mathModel.begin(); p != mathModel.end(); p++)
				{
					if (p->distance(*v) <= TOUCH_DISTANCE)
					{
						map<Polygon3D*, vector<ofVec3f> >::iterator it = touchPoints.find(&*p);
						if (it == touchPoints.end())
						{
							vector<ofVec3f> points;
							touchPoints.insert(make_pair(&*p, points));
							it = touchPoints.find(&*p);
						}
						it->second.push_back(p->project(*v));
						break;
					}
				}
			}
		}

		vector<ofVec3f> touchVecs;
		for (map<Polygon3D*, vector<ofVec3f> >::const_iterator it = touchPoints.begin(); it != touchPoints.end(); ++it)
		{
		}

		saveCloudAsFile("touchPoints.pcd", touchVecs);
	}

}
