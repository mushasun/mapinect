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
		isNewFrameAvailableMutex.lock();
		isNewFrameAvailable = detectMode;
		isNewFrameAvailableMutex.unlock();
	}

	//--------------------------------------------------------------
	void PCMThread::newForcedFrameAvailable()
	{
		isNewFrameAvailableMutex.lock();
		isNewForcedFrameAvailable = true;
		isNewFrameAvailableMutex.unlock();
	}

	//--------------------------------------------------------------
	void PCMThread::threadedFunction() {
		while (isThreadRunning()) {
			if (lock()) {
				
				bool newFrameAvailable = false;
				{
					isNewFrameAvailableMutex.lock();
					newFrameAvailable = isNewFrameAvailable || isNewForcedFrameAvailable;
					isNewFrameAvailable = false;
					isNewForcedFrameAvailable = false;
					isNewFrameAvailableMutex.unlock();
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

			Table::create(coefficients, biggestPlaneCloud);
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
		
		
		// Comentado para probar estabilidad en la diferencia de nube
		//int dif = getDifferencesCloud(modelsCloud, cloud, filteredCloud, Constants::CLOUD_VOXEL_SIZE);
		int dif = getDifferencesCloud(modelsCloud, cloud, filteredCloud, Constants::CLOUD_VOXEL_SIZE*3);
		
		
		
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
		if(objectDetection)
			objectsThread.setClouds(objectsOnTableTopCloud);

		if(IsFeatureActive(FEATURE_HAND_DETECTION) && touchDetection)
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
									if (distance <= touchDistance && distance < minDistance)
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
					bool usePCL = true;
					if (useClustering)
					{
						const double tolerance = Constants::TOUCH_TOLERANCE();
						const int minClusterSize = 1;
						if (usePCL)
						{
							std::vector<pcl::PointIndices> cluster_indices;
							if (polygonTouchPointsCloud->size() == minClusterSize)
							{
								cluster_indices.push_back(pcl::PointIndices());
								cluster_indices[0].indices.push_back(0);
							}
							else
							{
								cluster_indices = findClusters(polygonTouchPointsCloud, tolerance, minClusterSize);
							}

							for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
							{
								PCPtr cloudCluster = getCloudFromIndices(polygonTouchPointsCloud, *it);
								ofVec3f touchPoint(computeCentroid(cloudCluster));
								newTouchPoints.push_back(TrackedTouchPtr(new TrackedTouch(i->first, touchPoint)));
							}
						}
						else
						{
							// Arreglo para que tome de a un punto tambien. 
							vector<vector<ofVec3f> > clusters;
							if(polygonTouchPointsCloud->size() == minClusterSize)
								clusters.push_back(i->second);
							else
								clusters = findClusters(pointCloudToOfVecVector(polygonTouchPointsCloud), tolerance, minClusterSize);

							for (vector<vector<ofVec3f> >::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
							{
								ofVec3f touchPoint(computeCentroid(*it));
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
				
				for (list<TrackedTouchPtr>::iterator tt = touchPointsToAdd.begin(); tt != touchPointsToAdd.end(); tt++)
				{
					int polygonId = (*tt)->getPolygonId();
					map<int, list<TrackedTouchPtr> >::iterator polygonTouchs = trackedTouchPoints.find(polygonId);
					if (polygonTouchs == trackedTouchPoints.end())
					{
						trackedTouchPoints.insert(make_pair(polygonId, list<TrackedTouchPtr>()));
						polygonTouchs = trackedTouchPoints.find(polygonId);
					}
					if (polygonTouchs->second.size() < Constants::TOUCH_MAX_PER_FACE)
					{
						polygonTouchs->second.push_back(*tt);
					}
				}
			}

			// Effectuate the update of the tracked touch points with the new ones
			setPCMThreadStatus("Updating touch points and pushing events to the application...");
			log(kLogFilePCMThread, "Updating touch points and pushing events to the application...");
			updateDetectedTouchPoints();

		}
	}

	//--------------------------------------------------------------
	bool PCMThread::findBestFit(const TrackedTouchPtr& tracked, TrackedTouchPtr& removed, bool &wasRemoved)
	{
		bool result = false;

		float currentDist = numeric_limits<float>::max();
		TrackedTouchPtr currentMatch;
		wasRemoved = false;
		map<int, list<TrackedTouchPtr> >::iterator polygonTouches = trackedTouchPoints.find(tracked->getPolygonId());
		if (polygonTouches != trackedTouchPoints.end())
		{
			for (list<TrackedTouchPtr>::iterator iter = polygonTouches->second.begin(); iter != polygonTouches->second.end(); iter++)
			{
				float dist = (*iter)->matchingTrackedTouch(tracked);
				if (dist < currentDist)
				{
					currentMatch = (*iter);
					currentDist = dist;
				}
			}
			if(currentDist < numeric_limits<float>::max())
			{
				wasRemoved = currentMatch->confirmMatch(tracked, removed);
				result = true;
			}
		}
		return result;
	}

	//--------------------------------------------------------------
	void PCMThread::updateDetectedTouchPoints()
	{
		for (map<int, list<TrackedTouchPtr> >::iterator p = trackedTouchPoints.begin(); p != trackedTouchPoints.end(); p++)
		{
			for (list<TrackedTouchPtr>::iterator iter = p->second.begin(); iter != p->second.end(); iter++)
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
			p->second.remove_if(isStatusReleased);

			for (list<TrackedTouchPtr>::iterator iter = p->second.begin(); iter != p->second.end(); iter++)
			{
				(*iter)->updateToHolding();
			}
		}
	}

}
