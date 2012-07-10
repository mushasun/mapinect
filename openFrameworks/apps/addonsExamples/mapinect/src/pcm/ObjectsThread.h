#ifndef OBJECTS_THREAD_H__
#define OBJECTS_THREAD_H__

#include "ofThread.h"

#include <list>

#include "ofxMutex.h"
#include "TrackedCloud.h"

namespace mapinect
{
	class ObjectsThread : ofThread
	{
	public:
		void						reset();
		void						setup();
		void						exit();
		virtual void				threadedFunction();

		void						setClouds(const PCPtr& cloud);

	private:
		void						processCloud();
		bool						findBestFit(const TrackedCloudPtr& trackedCloud, TrackedCloudPtr& removedCloud, bool &removed);
		void						updateDetectedObjects();
		vector<TrackedCloudPtr>		computeOcclusions(const vector<TrackedCloudPtr>& potentialOcclusions);

		PCPtr						inCloud;
		PCPtr						inRawCloud;

		ofxMutex					inCloudMutex;

		list<TrackedCloudPtr>		trackedClouds;

	};
}

#endif	// OBJECTS_THREAD_H__
