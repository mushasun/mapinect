#ifndef PCM_THREAD_H__
#define PCM_THREAD_H__

#include "ofThread.h"

#include <list>

#include "ofxMutex.h"
#include "ObjectsThread.h"
#include "TrackedTouch.h"

namespace mapinect
{
	class PCMThread : ofThread
	{
	public:
		PCMThread();

		void						reset();
		void						setup();
		void						exit();
		virtual void				threadedFunction();
		inline void					startDetection()		{ detectMode = true; }
		void						newFrameAvailable(bool forceDetection = false);
		void						newForcedFrameAvailable();

		void						processCloud();

	private:
		PCPtr						getObjectsOnTableTopCloud(PCPtr& occludersCloud);
		PCPtr						getDifferenceCloudFromModel(const PCPtr& cloud);

		bool						findBestFit(const TrackedTouchPtr& tracked, TrackedTouchPtr& removed, bool &wasRemoved);
		void						updateDetectedTouchPoints();

		list<TrackedTouchPtr>		trackedTouchPoints;

		bool						detectMode;
		bool						isNewFrameAvailable;
		bool						isNewForcedFrameAvailable;
		ofxMutex					isNewFrameAvailableMutex;

		ofVec3f						tableClusterLastCentroid;

		ObjectsThread				objectsThread;
	};
}

#endif	// PCM_THREAD_H__
