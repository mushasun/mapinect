#ifndef MAPINECT_TRACKEDCLOUD_H__
#define MAPINECT_TRACKEDCLOUD_H__

#include <pcl/point_cloud.h>
#include "PCModelObject.h"
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace pcl;

typedef pcl::PointCloud<pcl::Normal>				SurfaceNormals;
typedef SurfaceNormals::Ptr							SurfaceNormalsPtr;
typedef pcl::PointCloud<pcl::FPFHSignature33>		LocalFeatures;
typedef LocalFeatures::Ptr							LocalFeaturesPtr;
typedef pcl::search::KdTree<pcl::PointXYZ>			SearchMethod;
typedef SearchMethod::Ptr							SearchMethodPtr;

namespace mapinect {

	class TrackedCloud;

	typedef boost::shared_ptr<TrackedCloud> TrackedCloudPtr;

	class TrackedCloud {
	
	public:
		TrackedCloud();
		TrackedCloud(const PCPtr& cloud);
		TrackedCloud(const PCPtr& cloud, bool isHand, bool forceCreate);
		virtual ~TrackedCloud();

		//float	matches(const TrackedCloudPtr& trackedCloud);
		bool	confirmMatch(const TrackedCloudPtr& trackedCloud, TrackedCloudPtr& removedCloud);
		float	matchingTrackedObjects(const TrackedCloudPtr& tracked_temp);

		void addCounter(int diff);
		void updateMatching();
		inline int getCounter() { return counter; }
		inline const PCPtr& getTrackedCloud() { return cloud; }
		inline bool hasObject() { return objectInModel != NULL; }
		inline bool hasMatching() { return matchingCloud != NULL; }
		inline bool isPotentialHand() { return hand; }
		void removeMatching();
		void updateCloud(const PCPtr& cloud_cluster);
		
		bool operator==(const TrackedCloudPtr& other) const ;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr getLocalFeatures ();
		inline const PCModelObjectPtr& getTrackedObject() { return objectInModel; }
		//bool matchingTrackedObjects(TrackedCloud tracked_temp, TrackedCloud tracked_obj, Eigen::Affine3f &transformation);
	protected:
		// Compute the surface normals and local features
		void processInput ();

		// Compute the surface normals
		void computeSurfaceNormals ();

		// Compute the local feature descriptors
		void computeLocalFeatures ();

	private:
		void						init();

		PCPtr						cloud;
		int							counter;
		TrackedCloudPtr				matchingCloud;
		PCModelObjectPtr			objectInModel;
		int							minPointDif;
		float						nearest;
		bool						needApplyTransformation;
		bool						needRecalculateFaces;
		bool						hand;

		SurfaceNormalsPtr			normals;
		LocalFeaturesPtr			features;
		SearchMethodPtr				search_method_xyz;
		bool						features_computed;
		// Parameters
		float						normal_radius;
		float						feature_radius;
	};

}
#endif	// MAPINECT_TRACKEDCLOUD_H__
