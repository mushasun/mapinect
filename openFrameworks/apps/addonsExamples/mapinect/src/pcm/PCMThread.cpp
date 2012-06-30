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
		PCPtr result(new PC());

		PCPtr cloud = getCloud();
		saveCloud("rawCloud.pcd", *cloud);
		
		if (cloud->size() == 0)
		{
			return result;
		}

		pcl::ModelCoefficients coefficients;
		bool tableSetted = false;
		{
			gModel->tableMutex.lock();
			if (gModel->getTable().get() != NULL)
			{
				tableSetted = true;
				coefficients = gModel->getTable()->getCoefficients();
			}
			gModel->tableMutex.unlock();
		}

		if (!tableSetted)
		{
			setPCMThreadStatus("Scanning table cluster...");
			log(kLogFilePCMThread, "Scanning table cluster...");
			std::vector<pcl::PointIndices> cluster_indices =
				findClusters(cloud, MAX_TABLE_CLUSTER_TOLERANCE, MIN_TABLE_CLUSTER_SIZE, MAX_TABLE_CLUSTER_SIZE);

			PCPtr tableCluster;
			float minDistanceToCentroid = MAX_FLOAT;
			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
			{
				PCPtr cloud_cluster = getCloudFromIndices(cloud, *it);
				ofVec3f ptoCentroid = computeCentroid(cloud_cluster);
				if (ptoCentroid.squareLength() < minDistanceToCentroid)
				{
					minDistanceToCentroid = ptoCentroid.squareLength();
					tableCluster = cloud_cluster;
				}
			}

			PCPtr biggestPlaneCloud = extractBiggestPlane(tableCluster, coefficients, result, 0.009);

			Table::Create(coefficients, biggestPlaneCloud);
		}
		else
		{
			setPCMThreadStatus("Removing table...");
			log(kLogFilePCMThread, "Removing table...");

			Polygon3D tableModel;
			{
				gModel->tableMutex.lock();
				tableModel = gModel->getTable()->getMathPolygonModelApproximation()->getMathModel();
				gModel->tableMutex.unlock();
			}

			for (PC::const_iterator p = cloud->begin(); p != cloud->end(); ++p)
			{
				if (evaluatePoint(coefficients, *p) > OCTREE_RES * 2)
				{
					if (tableModel.isInPolygon(tableModel.getPlane().project(PCXYZ_OFVEC3F((*p)))))
					{
						result->push_back(*p);
					}
				}
			}
		}

		return result;
	}

	PCPtr PCMThread::getDifferenceCloudFromModel(const PCPtr& cloud)
	{
		//saveCloudAsFile("sourceCloud.pcd", *cloud);

		PCPtr modelsCloud(gModel->getCloudSum());
		//saveCloudAsFile("modelsCloud.pcd", *modelsCloud);

		PCPtr filteredCloud(new PointCloud<PointXYZ>);
		int dif = getDifferencesCloud(modelsCloud, cloud, filteredCloud, CLOUD_RES * 0.003);
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
			if (objectsOnTableTopCloud->size() > 200)
			{
				PCPtr differenceCloud = getDifferenceCloudFromModel(objectsOnTableTopCloud);
				vector<IObjectPtr> mathModel(gModel->getMathModelApproximation());

				// touch detection and tracking

			setPCMThreadStatus("Detecting touch points...");
			log(kLogFilePCMThread, "Detecting touch points...");
			const float clusterTolerance = CLOUD_RES * 0.003 * 1.73205f;
			const int clusterMinSize = 50;
			const int clusterMaxSize = 5000;
			vector<pcl::PointIndices> clusterIndices = findClusters(differenceCloud, clusterTolerance, clusterMinSize, clusterMaxSize);
		
				map<IPolygonPtr, vector<ofVec3f> > pointsCloserToModel;

				for (int i = 0; i < clusterIndices.size(); ++i)
				{
					PCPtr cluster = getCloudFromIndices(differenceCloud, clusterIndices.at(i));
					saveCloud("cluster" + ofToString(i) + ".pcd", *cluster);

					// filter the points that are close to the math model
					vector<ofVec3f> vCluster(pointCloudToOfVecVector(cluster));
					for (vector<ofVec3f>::iterator v = vCluster.begin(); v != vCluster.end(); ++v)
					{
						bool found = false;
						for (vector<IObjectPtr>::const_iterator ob = mathModel.begin(); !found && ob != mathModel.end(); ++ob)
						{
							float minDistance = MAX_FLOAT;
							IPolygonPtr polygon;
							for (vector<IPolygonPtr>::const_iterator p = (*ob)->getPolygons().begin(); p != (*ob)->getPolygons().end(); ++p)
							{
								ofVec3f planeProjected((*p)->getMathModel().getPlane().project(*v));
								if ((*p)->getMathModel().isInPolygon(planeProjected))
								{
									float distance = (*p)->getMathModel().distance(*v);
									if (inRange(distance, TOUCH_DISTANCE / 4.0f, TOUCH_DISTANCE) && distance < minDistance)
									{
										minDistance = distance;
										polygon = *p;
									}
								}
							}
							if (polygon.get() != NULL)
							{
								found = true;
								map<IPolygonPtr, vector<ofVec3f> >::iterator it = pointsCloserToModel.find(polygon);
								if (it == pointsCloserToModel.end())
								{
									vector<ofVec3f> points;
									pointsCloserToModel.insert(make_pair(polygon, points));
									it = pointsCloserToModel.find(polygon);
								}
								it->second.push_back((polygon)->getMathModel().getPlane().project(*v));
							}
						}
					}
				}

				list<TrackedTouchPtr> newTouchPoints;
				for (map<IPolygonPtr, vector<ofVec3f> >::const_iterator i = pointsCloserToModel.begin(); i != pointsCloserToModel.end(); ++i)
				{
					PCPtr polygonTouchPointsCloud(ofVecVectorToPointCloud(i->second));
					bool useClustering = true;
					bool usePCL = false;
					if (useClustering)
					{
						const double tolerance = MAX_CLUSTER_TOLERANCE / 2;
						const int minClusterSize = 1;
						const int maxClusterSize = 5000;
						const int maxTouchClusterSize = 8;
						if (usePCL)
						{
							std::vector<pcl::PointIndices> cluster_indices =
								findClusters(polygonTouchPointsCloud, tolerance, minClusterSize, maxClusterSize);

							for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
							{
								if (it->indices.size() <= maxTouchClusterSize)		// Avoid new big objects from being recognized as touch points
								{
									PCPtr cloud_cluster = getCloudFromIndices(polygonTouchPointsCloud, *it);
									ofVec3f touchPoint(computeCentroid(cloud_cluster));
									newTouchPoints.push_back(TrackedTouchPtr(new TrackedTouch(i->first, touchPoint)));
								}
							}
						}
						else
						{
							vector<vector<ofVec3f> > clusters = findClusters(pointCloudToOfVecVector(polygonTouchPointsCloud),
								tolerance, minClusterSize, maxClusterSize);

							for (vector<vector<ofVec3f> >::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
							{
								if (it->size() <= maxTouchClusterSize)
								{
									ofVec3f touchPoint(computeCentroid(*it));
									newTouchPoints.push_back(TrackedTouchPtr(new TrackedTouch(i->first, touchPoint)));
								}
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
			}

			// Effectuate the update of the tracked touch points with the new ones
			setPCMThreadStatus("Updating touch points and pushing events to the application...");
			log(kLogFilePCMThread, "Updating touch points and pushing events to the application...");
			updateDetectedTouchPoints();

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
			if ((*iter)->updateMatching())
			{
				EventManager::addEvent(
					MapinectEvent(kMapinectEventTypeObjectTouched,
						(*iter)->getObject(),
						(*iter)->getDataTouch()));
			}
		}
		// Clear released touch points
		trackedTouchPoints.remove_if(isStatusReleased);

		for (list<TrackedTouchPtr>::iterator iter = trackedTouchPoints.begin(); iter != trackedTouchPoints.end(); iter++)
		{
			(*iter)->updateToHolding();
		}
	}

}
