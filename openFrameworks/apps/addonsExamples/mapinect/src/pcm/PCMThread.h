#ifndef PCM_THREAD_H__
#define PCM_THREAD_H__

#include "ofThread.h"

#include <list>

#include "ObjectsThread.h"
#include "Polygon3D.h"
#include "ofxMutex.h"

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
		void						newFrameAvailable();
		void						newForcedFrameAvailable();

		void						processCloud();

	private:
		PCPtr						getObjectsOnTableTopCloud();
		PCPtr						getDifferenceCloudFromModel(const PCPtr& cloud, vector<Polygon3D>& mathModel);

		bool						detectMode;
		bool						isNewFrameAvailable;
		bool						isNewForcedFrameAvailable;
		ofxMutex					isNewFrameAvailableMutex;

		ofVec3f						tableClusterLastCentroid;

		ObjectsThread				objectsThread;
	};
}

#endif	// PCM_THREAD_H__
