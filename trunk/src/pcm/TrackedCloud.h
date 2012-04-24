#ifndef MAPINECT_TRACKEDCLOUD_H__
#define MAPINECT_TRACKEDCLOUD_H__

#include <pcl/point_cloud.h>
#include "PCModelObject.h"
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace pcl;
namespace mapinect {
	class TrackedCloud {
	
	public:
		typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
		typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
		typedef pcl::KdTreeFLANN<pcl::PointXYZ> SearchMethod;
		TrackedCloud(PointCloud<PointXYZ>::Ptr cloud);
		TrackedCloud(PointCloud<PointXYZ>::Ptr cloud, bool isHand, bool forceCreate);
		TrackedCloud();
		virtual ~TrackedCloud();

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
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr getLocalFeatures ();
		//bool matchingTrackedObjects(TrackedCloud tracked_temp, TrackedCloud tracked_obj, Eigen::Affine3f &transformation);
	protected:
		// Compute the surface normals and local features
		void processInput ();

		// Compute the surface normals
		void computeSurfaceNormals ();

		// Compute the local feature descriptors
		void computeLocalFeatures ();
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

		pcl::PointCloud<pcl::Normal>::Ptr				normals_;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr		features_;
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr			search_method_xyz_;
		bool						features_computed;
		// Parameters
		float normal_radius_;
		float feature_radius_;
	};
}
#endif	// MAPINECT_TRACKEDCLOUD_H__
