#include "PCMThread.h"

#include "Constants.h"
#include "Globals.h"
#include "log.h"
#include "pointUtils.h"

using namespace std;

#define WAIT_TIME_MS		40

namespace mapinect {

	//--------------------------------------------------------------
	void ObjectsThread::reset()
	{
		trackedClouds.clear();
		gModel->resetObjects();
	}

	//--------------------------------------------------------------
	void ObjectsThread::setup()
	{
		// Start the cloud processing thread
		startThread(true, false);
	}

	//--------------------------------------------------------------
	void ObjectsThread::exit()
	{
		stopThread();
	}

	//--------------------------------------------------------------
	void ObjectsThread::setCloud(const PCPtr& cloud)
	{
		inCloudMutex.lock();
		inCloud = cloud;
		inCloudMutex.unlock();
	}

	//--------------------------------------------------------------
	void ObjectsThread::threadedFunction()
	{
		while (isThreadRunning()) {
			if (lock()) {
				
				bool newCloudAvailable = false;
				{
					inCloudMutex.lock();
					if (inCloud.get() != NULL)
						newCloudAvailable = true;
					inCloudMutex.unlock();

				}

				if(newCloudAvailable)
				{
					processCloud();
					setObjectsThreadStatus("");
				}
				
				unlock();
				ofSleepMillis(WAIT_TIME_MS);
			}
		}
	}
	
	//--------------------------------------------------------------
	bool countIsZero(const TrackedCloudPtr &trackedCloud)
	{
		return trackedCloud->getCounter() == 0;
	}

	//--------------------------------------------------------------
	void ObjectsThread::processCloud()
	{
		PCPtr cloud;
		{
			inCloudMutex.lock();
			cloud = PCPtr(new PC(*inCloud));
			inCloud.reset();
			inCloudMutex.unlock();
		}

		// Updating temporal detections
		for (list<TrackedCloudPtr>::iterator iter = trackedClouds.begin(); iter != trackedClouds.end(); iter++) {
			(*iter)->addCounter(-1);
		}
		trackedClouds.remove_if(countIsZero);
		
		if(cloud->empty())
		{
			return;
		}

		list<TrackedCloudPtr> newClouds;
		int debugCounter = 0;

		{
				setObjectsThreadStatus("Detecting clusters...");
				std::vector<pcl::PointIndices> cluster_indices =
				findClusters(cloud, MAX_CLUSTER_TOLERANCE, MIN_CLUSTER_SIZE, MAX_CLUSTER_SIZE);

			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
			{
				PCPtr cloud_cluster = getCloudFromIndices(cloud, *it);

				saveCloudAsFile("objectsCluster" + ofToString(debugCounter) + ".pcd", *cloud_cluster);

				newClouds.push_back(TrackedCloudPtr(new TrackedCloud(cloud_cluster)));
				debugCounter++;
			}
		}

		// Look into the new clouds for the best fit
		list<TrackedCloudPtr> cloudsToMatch;
		list<TrackedCloudPtr> cloudsToAdd;

		debugCounter = 0;
		int max_iter = 10;
		setObjectsThreadStatus("Matching clusters with existing ones...");
		do
		{
			for (list<TrackedCloudPtr>::iterator iter = newClouds.begin(); iter != newClouds.end(); iter++)
			{
				//saveCloudAsFile("clusterInTable" + ofToString(debugCounter) + ".pcd", *(*iter)->getTrackedCloud());
				TrackedCloudPtr removedCloud;
				bool removed = false;
				bool fitted = findBestFit(*iter, removedCloud, removed);

				if (removed)
					cloudsToMatch.push_back(removedCloud);	// Push back the old cloud to try again
				if (!fitted)
					cloudsToAdd.push_back(*iter);			// No matching cloud, this will be a new object
				debugCounter++;
			}
			newClouds = cloudsToMatch;
			cloudsToMatch.clear();
			max_iter--;
		}
		while (newClouds.size() > 0 && max_iter > 0);

		// Effectuate the update of the tracked cloud with the new ones
		setObjectsThreadStatus("Update existing and new data...");
		updateDetectedObjects();

		trackedClouds.insert(trackedClouds.end(), cloudsToAdd.begin(), cloudsToAdd.end());
	}

	//--------------------------------------------------------------
	bool ObjectsThread::findBestFit(const TrackedCloudPtr& trackedCloud, TrackedCloudPtr& removedCloud, bool &removed) {
		for (list<TrackedCloudPtr>::iterator iter = trackedClouds.begin(); iter != trackedClouds.end(); iter++) {
			if ((*iter)->matches(trackedCloud, removedCloud, removed))
			{
				return true;
			}
		}
		return false;
	}

	//--------------------------------------------------------------
	void ObjectsThread::updateDetectedObjects()
	{
		for (list<TrackedCloudPtr>::iterator iter = trackedClouds.begin(); iter != trackedClouds.end(); iter++) {
			if ((*iter)->hasMatching())
			{
				(*iter)->updateMatching();
				if (!((*iter)->hasObject())) {
					(*iter)->addCounter(2);
				}
				else
					(*iter)->addCounter(1);
				(*iter)->removeMatching();
			}
		}
	}

}
