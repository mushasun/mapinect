#include "PCMThread.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>

#include "ofxXmlSettings.h"

#include "Constants.h"
#include "EventManager.h"
#include "Globals.h"
#include "log.h"
#include "pointUtils.h"
#include "Feature.h"

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
	void PCMThread::newFrameAvailable(bool forceDetection)
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
					processCloud();
					setPCMThreadStatus("");
				}
				
				unlock();
				ofSleepMillis(WAIT_TIME_MS);
			}
		}
	}
	
	//--------------------------------------------------------------
	PCPtr PCMThread::getObjectsOnTableTopCloud(PCPtr &occludersCloud){
		PCPtr cloud = getCloud();
		//saveCloudAsFile("rawCloud.pcd", *cloud);
		
		if (cloud->size() == 0) {
			return cloud;
		}

		//Separo en clusters
		PCPtr tableCluster (new PC());

		setPCMThreadStatus("Searching table cluster...");
		log(kLogFilePCMThread, "Searching table cluster...");
		std::vector<pcl::PointIndices> cluster_indices =
			findClusters(cloud, MAX_TABLE_CLUSTER_TOLERANCE, MIN_TABLE_CLUSTER_SIZE, MAX_TABLE_CLUSTER_SIZE);

		//Busco el cluster más cercano y guardo posibles occluders
		int count = 1;
		float min_dist2 = MAX_FLOAT;
		ofVec3f newCentroid(tableClusterLastCentroid);
		vector<PCPtr> potentialOccluders;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			PCPtr cloud_cluster = getCloudFromIndices(cloud, *it);
			potentialOccluders.push_back(cloud_cluster);

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
		setPCMThreadStatus("Creating objects over table cloud...");
		log(kLogFilePCMThread, "Creating objects over table cloud...");

		{
			gModel->tableMutex.lock();
			if (gModel->getTable().get() == NULL)
			{
				PCPtr biggestPlaneCloud = extractBiggestPlane(tableCluster, coefficients, remainingCloud, 0.009);
				saveCloudAsFile("table.pcd", *biggestPlaneCloud);
				TablePtr table = TablePtr(new Table(coefficients, biggestPlaneCloud));
				table->detect();
				//table->setDrawPointCloud(false);
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

			//Filtro occluders
			for(int i = 0; i < potentialOccluders.size(); i++)
			{
				if(potentialOccluders.at(i) != tableCluster &&
				   gModel->getTable()->isOverTable(potentialOccluders.at(i)))
				   *occludersCloud += *potentialOccluders.at(i);
			}
			*occludersCloud += *remainingCloud;
			gModel->tableMutex.unlock();

			saveCloudAsFile("occludersCluster.pcd", *occludersCloud);

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

	PCPtr PCMThread::getDifferenceCloudFromModel(const PCPtr& cloud)
	{
		//saveCloudAsFile("sourceCloud.pcd", *cloud);

		PCPtr modelsCloud(gModel->getCloudSum());
		//saveCloudAsFile("modelsCloud.pcd", *modelsCloud);

		PCPtr filteredCloud(new PointCloud<PointXYZ>);
		int dif = getDifferencesCloud(modelsCloud, cloud, filteredCloud, 0.01);
		//cout << "Differences count: " << ofToString(dif) << endl;

		//saveCloudAsFile("dif.pcd", *filteredCloud);

		return filteredCloud;
	}

	//--------------------------------------------------------------
	bool isStatusReleased(const TrackedTouchPtr& trackedTouchPoint)
	{
		return trackedTouchPoint->getTouchStatus() == kTouchTypeReleased;
	}

	//--------------------------------------------------------------
	void PCMThread::processCloud()
	{
		bool dif;
		PCPtr occluders (new PC());
		PCPtr objectsOnTableTopCloud = getObjectsOnTableTopCloud(occluders);

		objectsThread.setClouds(objectsOnTableTopCloud,occluders);

		if(IsFeatureActive(FEATURE_HAND_DETECTION))
		{
			// split the new cloud from the existing one
			setPCMThreadStatus("Obtaining difference cloud from model...");
			log(kLogFilePCMThread, "Obtaining difference cloud from model...");
			PCPtr differenceCloud = getDifferenceCloudFromModel(objectsOnTableTopCloud);
			vector<IObjectPtr> mathModel(gModel->getMathModelApproximation());
		
			// touch detection and tracking

			setPCMThreadStatus("Detecting touch points...");
			log(kLogFilePCMThread, "Detecting touch points...");
			vector<pcl::PointIndices> clusterIndices = findClusters(differenceCloud, MAX_CLUSTER_TOLERANCE, 100, 5000);
		
			map<IPolygonPtr, vector<ofVec3f> > pointsCloserToModel;

			for (int i = 0; i < clusterIndices.size(); ++i)
			{
				PCPtr cluster = getCloudFromIndices(differenceCloud, clusterIndices.at(i));
				saveCloudAsFile("cluster" + ofToString(i) + ".pcd", *cluster);

				// filter the points that are close to the math model
				vector<ofVec3f> vCluster(pointCloudToOfVecVector(cluster));
				for (vector<ofVec3f>::iterator v = vCluster.begin(); v != vCluster.end(); ++v)
				{
					for (vector<IObjectPtr>::const_iterator ob = mathModel.begin(); ob != mathModel.end(); ++ob)
					{
						for (vector<IPolygonPtr>::const_iterator p = (*ob)->getPolygons().begin(); p != (*ob)->getPolygons().end(); ++p)
						{
							if ((*p)->getMathModel().distance(*v) <= TOUCH_DISTANCE)
							{
								map<IPolygonPtr, vector<ofVec3f> >::iterator it = pointsCloserToModel.find(*p);
								if (it == pointsCloserToModel.end())
								{
									vector<ofVec3f> points;
									pointsCloserToModel.insert(make_pair(*p, points));
									it = pointsCloserToModel.find(*p);
								}
								it->second.push_back((*p)->getMathModel().project(*v));
								break;
							}
						}
					}
				}
			}

			list<TrackedTouchPtr> newTouchPoints;
			for (map<IPolygonPtr, vector<ofVec3f> >::const_iterator i = pointsCloserToModel.begin(); i != pointsCloserToModel.end(); ++i)
			{
				PCPtr polygonTouchPointsCloud(ofVecVectorToPointCloud(i->second));
				bool useClustering = false;
				if (useClustering)
				{
					std::vector<pcl::PointIndices> cluster_indices =
						findClusters(polygonTouchPointsCloud, 0.01, 2, 5000);

					for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
					{
						if (it->indices.size() <= 10)		// Avoid new big objects from being recognized as touch points
						{
							PCPtr cloud_cluster = getCloudFromIndices(polygonTouchPointsCloud, *it);
							ofVec3f touchPoint(computeCentroid(cloud_cluster));
							newTouchPoints.push_back(TrackedTouchPtr(new TrackedTouch(i->first, touchPoint)));
						}
					}
				}
				else
				{
					for (vector<ofVec3f>::const_iterator v = i->second.begin(); v != i->second.end(); ++v)
					{
						newTouchPoints.push_back(TrackedTouchPtr(new TrackedTouch(i->first, *v)));
					}
				}
			}

			// Look into the new touch points for the best fit
			list<TrackedTouchPtr> touchPointsToMatch;
			list<TrackedTouchPtr> touchPointsToAdd;

			int max_iter = 10;
			setPCMThreadStatus("Matching touch points with existing ones...");
			log(kLogFilePCMThread, "Matching touch points with existing ones...");
			do
			{
				for (list<TrackedTouchPtr>::iterator iter = newTouchPoints.begin(); iter != newTouchPoints.end(); iter++)
				{
					TrackedTouchPtr removed;
					bool wasRemoved = false;
					bool fitted = findBestFit(*iter, removed, wasRemoved);

					if (wasRemoved)
						touchPointsToMatch.push_back(removed);	// Push back the old cloud to try again
					if (!fitted)
						touchPointsToAdd.push_back(*iter);			// No matching cloud, this will be a new object
				}
				newTouchPoints = touchPointsToMatch;
				touchPointsToMatch.clear();
				max_iter--;
			}
			while (newTouchPoints.size() > 0 && max_iter > 0);

			trackedTouchPoints.insert(trackedTouchPoints.end(), touchPointsToAdd.begin(), touchPointsToAdd.end());

			// Effectuate the update of the tracked touch points with the new ones
			setPCMThreadStatus("Updating touch points and pushing events to the application...");
			log(kLogFilePCMThread, "Updating touch points and pushing events to the application...");
			updateDetectedTouchPoints();

			// Clear released touch points
			trackedTouchPoints.remove_if(isStatusReleased);
		}
	}

	//--------------------------------------------------------------
	bool PCMThread::findBestFit(const TrackedTouchPtr& tracked, TrackedTouchPtr& removed, bool &wasRemoved) {
		for (list<TrackedTouchPtr>::iterator iter = trackedTouchPoints.begin(); iter != trackedTouchPoints.end(); iter++) {
			if ((*iter)->matches(tracked, removed, wasRemoved))
			{
				return true;
			}
		}
		return false;
	}

	//--------------------------------------------------------------
	void PCMThread::updateDetectedTouchPoints()
	{
		for (list<TrackedTouchPtr>::iterator iter = trackedTouchPoints.begin(); iter != trackedTouchPoints.end(); iter++)
		{
			(*iter)->updateMatching();
			EventManager::addEvent(
				MapinectEvent(kMapinectEventTypeObjectTouched,
					(*iter)->getObject(),
					(*iter)->getDataTouch()));
		}
	}

}
