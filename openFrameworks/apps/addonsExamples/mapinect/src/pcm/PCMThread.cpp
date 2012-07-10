#include "PCMThread.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>

#include "Constants.h"
#include "EventManager.h"
#include "Globals.h"
#include "log.h"
#include "pointUtils.h"
#include "Feature.h"
#include "ButtonManager.h"

using namespace std;

#define WAIT_TIME_MS		20

namespace mapinect {
	PCMThread::PCMThread()
	{
		detectMode = false;
		isNewFrameAvailable = false;
		isNewForcedFrameAvailable = false;
	}

	void PCMThread::reset()
	{
		Constants::LoadConstants();
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
					isNewFrameAvailable = false;
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

		setPCMThreadStatus("Obtaining cloud...");
		log(kLogFilePCMThread, "Obtaining cloud...");
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
				findClusters(cloud, Constants::TABLE_CLUSTER_TOLERANCE(), Constants::TABLE_CLUSTER_MIN_SIZE());

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
				if (evaluatePoint(coefficients, *p) > Constants::TABLE_HEIGHT_TOLERANCE())
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
		int dif = getDifferencesCloud(modelsCloud, cloud, filteredCloud, Constants::CLOUD_VOXEL_SIZE);
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

		objectsThread.setClouds(objectsOnTableTopCloud);

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
			const float touchDistance = Constants::TOUCH_DISTANCE();
			const float clusterTolerance = Constants::OBJECT_CLUSTER_TOLERANCE();
			const int clusterMinSize = Constants::TOUCH_CLUSTER_MIN_SIZE();
			vector<pcl::PointIndices> clusterIndices = findClusters(differenceCloud, clusterTolerance, clusterMinSize);
		
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
									if (inRange(distance, touchDistance / 4.0f, touchDistance) && distance < minDistance)
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
						const double tolerance = touchDistance / 2.0f;
						const int minClusterSize = 1;
						const int maxTouchClusterSize = Constants::TOUCH_MAX_PER_FACE;
						if (usePCL)
						{
							std::vector<pcl::PointIndices> cluster_indices =
								findClusters(polygonTouchPointsCloud, tolerance, minClusterSize);

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
							vector<vector<ofVec3f> > clusters =
									findClusters(pointCloudToOfVecVector(polygonTouchPointsCloud), tolerance, minClusterSize);

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
				//TODO: refinar cuando se envian estas senales
				
				/*EventManager::addEvent(
					MapinectEvent(kMapinectEventTypeButtonPressed,
					(*iter)->getDataTouch()));*/
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
