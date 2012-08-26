#ifndef MAPINECT_TRACKEDCLOUD_H__
#define MAPINECT_TRACKEDCLOUD_H__

#include "PCModelObject.h"

using namespace pcl;

namespace mapinect {

	//class TrackedCloud;

	//typedef boost::shared_ptr<TrackedCloud> TrackedCloudPtr;

	class TrackedCloud {
	
	public:
		TrackedCloud();
		TrackedCloud(const PCPtr& cloud);
		virtual ~TrackedCloud();

		bool							confirmMatch(const TrackedCloudPtr& trackedCloud, TrackedCloudPtr& removedCloud);
		float							matchingTrackedObjects(const TrackedCloudPtr& trackedTemp) const;

		void							addCounter(int diff);
		void							updateMatching();
		inline int						getCounter() const		{ return counter; }
		inline const PCPtr&				getTrackedCloud() const	{ return cloud; }
		inline bool						hasObject() const		{ return objectInModel != NULL; }
		inline bool						hasMatching() const		{ return matchingCloud != NULL; }
		void							removeMatching();
		void							updateCloud(const PCPtr& cloudCluster);
		
		bool							operator==(const TrackedCloudPtr& other) const;
		inline const PCModelObjectPtr&	getTrackedObject()		{ return objectInModel; }

		inline bool						isInViewField() const		{ return inViewField; }
		inline void						setInViewField(bool val)    { inViewField = val; }
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
		int								invalidCounter;
		bool							inViewField;
	};

}
#endif	// MAPINECT_TRACKEDCLOUD_H__
