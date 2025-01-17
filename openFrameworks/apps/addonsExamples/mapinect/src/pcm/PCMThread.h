#ifndef PCM_THREAD_H__
#define PCM_THREAD_H__

#include "ofThread.h"

#include <list>

#include "ofxMutex.h"
#include "ObjectsThread.h"
#include "TrackedTouch.h"
#include "ButtonManager.h"

namespace mapinect
{
	class PCMThread : ofThread
	{
	public:
		PCMThread();

		void						reset();
		void						setup(ButtonManager* btnManager);
		void						exit();
		virtual void				threadedFunction();
		inline void					startDetection()					{ detectMode = true; }
		inline void					setObjectDetection(bool value)		{ objectDetection = value; }
		inline bool					getObjectDetection()				{ return objectDetection; }
		inline void					setTouchDetection(bool value)		{ touchDetection = value; }
		inline bool					getTouchDetection()					{ return touchDetection; }

		void						newFrameAvailable(bool forceDetection = false);
		void						newForcedFrameAvailable();

		void						processCloud();

	private:
		PCPtr						getObjectsOnTableTopCloud(PCPtr& occludersCloud);
		PCPtr						getDifferenceCloudFromModel(const PCPtr& cloud);

		bool						findBestFit(const TrackedTouchPtr& tracked, TrackedTouchPtr& removed, bool &wasRemoved);
		void						updateDetectedTouchPoints();

		map<int, list<TrackedTouchPtr> >	trackedTouchPoints;

		bool						detectMode;
		bool						objectDetection;
		bool						touchDetection;
		bool						isNewFrameAvailable;
		bool						isNewForcedFrameAvailable;
		ofxMutex					isNewFrameAvailableMutex;

		ofVec3f						tableClusterLastCentroid;

		ObjectsThread				objectsThread;
		ButtonManager*				btnManager;
	};
}

#endif	// PCM_THREAD_H__
