#ifndef MAPINECT_TRACKEDCLOUD_H__
#define MAPINECT_TRACKEDCLOUD_H__

#include "PCModelObject.h"

using namespace pcl;

namespace mapinect {

	class TrackedCloud;

	typedef boost::shared_ptr<TrackedCloud> TrackedCloudPtr;

	class TrackedCloud {
	
	public:
		TrackedCloud();
		TrackedCloud(const PCPtr& cloud);
		virtual ~TrackedCloud();

		bool							confirmMatch(const TrackedCloudPtr& trackedCloud, TrackedCloudPtr& removedCloud);
		float							matchingTrackedObjects(const TrackedCloudPtr& tracked_temp);

		void							addCounter(int diff);
		void							updateMatching();
		inline int						getCounter()			{ return counter; }
		inline const PCPtr&				getTrackedCloud()		{ return cloud; }
		inline bool						hasObject()				{ return objectInModel != NULL; }
		inline bool						hasMatching()			{ return matchingCloud != NULL; }
		void							removeMatching();
		void							updateCloud(const PCPtr& cloud_cluster);
		
		bool							operator==(const TrackedCloudPtr& other) const ;
		inline const PCModelObjectPtr&	getTrackedObject()		{ return objectInModel; }

	private:
		void							init();

		PCPtr							cloud;
		int								counter;
		TrackedCloudPtr					matchingCloud;
		PCModelObjectPtr				objectInModel;
		int								minPointDif;
		float							nearest;
		bool							needApplyTransformation;
		bool							needRecalculateFaces;
		ofVec3f							translationV;
	};

}
#endif	// MAPINECT_TRACKEDCLOUD_H__
