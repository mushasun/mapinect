#include "PCMThread.h"

#include "Constants.h"
#include "Globals.h"
#include "log.h"
#include "pointUtils.h"
#include "PCPolyhedron.h"
#include <pcl/octree/octree.h>

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
	void ObjectsThread::setClouds(const PCPtr& cloud, const PCPtr& rawCloud)
	{
		inCloudMutex.lock();
		inCloud = cloud;
		inRawCloud = rawCloud;
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
					if (inCloud.get() != NULL && inRawCloud.get() != NULL )
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
			updateDetectedObjects();
			return;
		}

		list<TrackedCloudPtr> newClouds;
		int debugCounter = 0;

		{
				setObjectsThreadStatus("Detecting clusters...");
				saveCloudAsFile("rawClusters.pcd", *cloud);
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
		float currentDist = numeric_limits<float>::max();
		TrackedCloudPtr currentMatch;
		removed = false;
		for (list<TrackedCloudPtr>::iterator iter = trackedClouds.begin(); iter != trackedClouds.end(); iter++) {
			float dist = (*iter)->matchingTrackedObjects(trackedCloud);
			if (dist != -1 && dist < currentDist)
			{
				currentMatch = (*iter);
				currentDist = dist;
			}
		}
		if(currentDist != numeric_limits<float>::max())
		{
			removed = currentMatch->confirmMatch(trackedCloud,removedCloud);
			return true;
		}
		return false;
	}

	//--------------------------------------------------------------
	vector<TrackedCloudPtr> ObjectsThread::computeOcclusions(const vector<TrackedCloudPtr>& potentialOcclusions)
	{
		vector<TrackedCloudPtr> occlusions;
		ofVec3f origin(0,0,0);

		inCloudMutex.lock();
		PCPtr cloud = PCPtr(new PC(*inRawCloud));
		inRawCloud.reset();
		inCloudMutex.unlock();

		saveCloudAsFile("rawInternal.pcd",*cloud);
		pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree(0.01);
		pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::AlignedPointTVector voxelList;

		octree.setInputCloud(cloud);
		octree.defineBoundingBox();
		octree.addPointsFromInputCloud();

		for(int i = 0; i < potentialOcclusions.size(); i++)
		{
			bool occluded = false;
			PCPolyhedron* polyhedron = dynamic_cast<PCPolyhedron*>(potentialOcclusions.at(i)->getTrackedObject().get());

			vector<ofVec3f> vexs = polyhedron->getVertexs();
			/*for(int o = 0; o < vexs.size() && !occluded; o++)
			{
				Line3D ray (vexs.at(o), ofVec3f(0,0,0));
				for(int j = 0; j < occluders.size() && !occluded; j ++)
				{
					vector<IPolygonPtr> pols  = occluders.at(i)->getTrackedObject()->getMathModelApproximation()->getPolygons();
					for(int k = 0; k < pols.size() && !occluded; k ++)
						occluded = pols.at(j)->getMathModel().isInPolygon(ray);
				}
				if(occluded)
					occlusions.push_back(potentialOcclusions.at(i));
			}*/
		
			for(int o = 0; o < vexs.size() && !occluded; o++)
			{
				ofVec3f end = vexs.at(o);
				Eigen::Vector3f endPoint(end.x,end.y,end.z);
				Eigen::Vector3f originPoint(0,0,0);
				voxelList.clear();

				int voxs = octree.getApproxIntersectedVoxelCentersBySegment(originPoint,endPoint,voxelList,0.01);

				for(int i = 0; i < voxelList.size(); i ++)
				{
					if(octree.isVoxelOccupiedAtPoint(voxelList.at(i)))
					{
						ofVec3f intersect (voxelList.at(i).x,voxelList.at(i).y,voxelList.at(i).z);
						if((intersect - origin).length() < (end - origin).length())
							occluded = true;
					}
				}

				if(occluded)
					occlusions.push_back(potentialOcclusions.at(i));
			}

		}
		return occlusions;
	}

	void ObjectsThread::updateDetectedObjects()
	{
		vector<TrackedCloudPtr> occluders;
		vector<TrackedCloudPtr> potentialOcclusions;
		vector<TrackedCloudPtr> occlusions;

		for (list<TrackedCloudPtr>::iterator iter = trackedClouds.begin(); iter != trackedClouds.end(); iter++) {
			if ((*iter)->hasMatching())
			{
				saveCloudAsFile("objPreMatched.pcd",*(*iter)->getTrackedCloud());
				(*iter)->updateMatching();
				saveCloudAsFile("objPostMatched.pcd",*(*iter)->getTrackedCloud());

				if (!((*iter)->hasObject())) {
					(*iter)->addCounter(2);
				}
				else
				{
					(*iter)->addCounter(1);
					occluders.push_back(*iter);
				}
				(*iter)->removeMatching();
			}
			else if ((*iter)->hasObject())
			{

				potentialOcclusions.push_back(*iter);
			}
		}

		if(potentialOcclusions.size() > 0)
		{
			occlusions = computeOcclusions(potentialOcclusions);
			for(int i = 0; i < occlusions.size(); i++)
			{
				occlusions.at(i)->addCounter(1);
			}
		}
	}
}
