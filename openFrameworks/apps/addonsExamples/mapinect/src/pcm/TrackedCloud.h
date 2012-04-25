#ifndef MAPINECT_TRACKEDCLOUD_H__
#define MAPINECT_TRACKEDCLOUD_H__

#include <pcl/point_cloud.h>
#include "PCModelObject.h"

using namespace pcl;
namespace mapinect {
	class TrackedCloud {
	private:
		PointCloud<PointXYZ>::Ptr	cloud;
		int							counter;
		TrackedCloud				*matchingCloud;
		PCModelObject				*objectInModel;
		int							minPointDif;
		float						nearest;
		bool						needApplyTransformation;
		bool						needRecalculateFaces;
		bool						hand;
	public:
		
		TrackedCloud(PointCloud<PointXYZ>::Ptr cloud);
		TrackedCloud(PointCloud<PointXYZ>::Ptr cloud, bool isHand);
		TrackedCloud();
		virtual ~TrackedCloud();

		bool matches(PointCloud<PointXYZ>::Ptr cloudCluster);
		bool matches(TrackedCloud* trackedCloud, TrackedCloud*& removedCloud, bool &removed);
		void addCounter(int diff);
		void updateMatching();
		inline int getCounter() { return counter; }
		inline PointCloud<PointXYZ>::Ptr getTrackedCloud() { return cloud; }
		inline bool hasObject() { return objectInModel != NULL; }
		inline bool hasMatching() { return matchingCloud != NULL; }
		inline bool isPotentialHand() { return hand; }
		void removeMatching();
		void updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster);
		bool matchingTrackedObjects(TrackedCloud tracked_temp, Eigen::Affine3f &transformation);
		bool operator==(const TrackedCloud &other) const ;
		//bool matchingTrackedObjects(TrackedCloud tracked_temp, TrackedCloud tracked_obj, Eigen::Affine3f &transformation);
	};
}
#endif	// MAPINECT_TRACKEDCLOUD_H__
