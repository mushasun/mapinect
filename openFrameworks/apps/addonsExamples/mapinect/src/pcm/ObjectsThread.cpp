#include "PCMThread.h"

#include "Constants.h"
#include "Globals.h"
#include "pointUtils.h"
#include "utils.h"

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
		ofxScopedMutex osm(inCloudMutex);
		inCloud = cloud;
	}

	//--------------------------------------------------------------
	void ObjectsThread::threadedFunction()
	{
		while (isThreadRunning()) {
			if (lock()) {
				
				bool newCloudAvailable = false;
				{
					ofxScopedMutex osm(inCloudMutex);
					if (inCloud.get() != NULL)
						newCloudAvailable = true;
				}

				if(newCloudAvailable)
				{
					processCloud();
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
			ofxScopedMutex osm(inCloudMutex);
			cloud = PCPtr(new PC(*inCloud));
			inCloud.reset();
		}

		//Actualizo las detecciones temporales
		for (list<TrackedCloudPtr>::iterator iter = trackedClouds.begin(); iter != trackedClouds.end(); iter++) {
			(*iter)->addCounter(-1);
		}
		trackedClouds.remove_if(countIsZero);
		
		if(cloud->empty())
		{
			return;
		}

		list<TrackedCloudPtr> nuevosClouds;
		int debugCounter = 0;

		{
			//Subdivido la nube de diferencias en clusters
			std::vector<pcl::PointIndices> cluster_indices =
				findClusters(cloud, MAX_CLUSTER_TOLERANCE, MIN_CLUSTER_SIZE, MAX_CLUSTER_SIZE);

			//separo en clusters
				
			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
			{
				PCPtr cloud_cluster = getCloudFromIndices(cloud, *it);

				saveCloudAsFile("objectsCluster" + ofToString(debugCounter) + ".pcd", *cloud_cluster);

				nuevosClouds.push_back(TrackedCloudPtr(new TrackedCloud(cloud_cluster)));
				debugCounter++;
			}
		}

		//Itero en todas las nuves encontradas buscando el mejor ajuste con un objeto encontrado
		list<TrackedCloudPtr> aProcesarClouds;
		list<TrackedCloudPtr> aAgregarClouds;

		debugCounter = 0;
		int max_iter = 10;
		do
		{
			for (list<TrackedCloudPtr>::iterator iter = nuevosClouds.begin(); iter != nuevosClouds.end(); iter++)
			{
				//saveCloudAsFile("clusterInTable" + ofToString(debugCounter) + ".pcd", *(*iter)->getTrackedCloud());
				TrackedCloudPtr removedCloud;
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

		for (list<TrackedCloudPtr>::iterator iter = aAgregarClouds.begin(); iter != aAgregarClouds.end(); iter++) {
			trackedClouds.push_back(TrackedCloudPtr(new TrackedCloud(**iter)));
		}
		////Los TrackedCloudPtr de aAgregarClouds quedan colgados?
		aAgregarClouds.clear();
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
