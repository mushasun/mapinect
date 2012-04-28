#include <limits>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include "TrackedCloud.h"

namespace mapinect {
class AlignmentDetector{
		private:
		TrackedCloud template_;
		TrackedCloud target_;

		// The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
		float min_sample_distance_;
		float max_correspondence_distance_;
		int nr_iterations_;

		public:
		// A struct for storing alignment results
		struct Result
		{
			float fitness_score;
			Eigen::Matrix4f final_transformation;
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		AlignmentDetector () :
		min_sample_distance_ (0.05f),
		max_correspondence_distance_ (0.1f*0.1f),
		nr_iterations_ (100)
		{
			// Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
			sac_ia_.setMinSampleDistance (min_sample_distance_);
			sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
			sac_ia_.setMaximumIterations (nr_iterations_);
		}

		~AlignmentDetector () {}

		// Set the given cloud as the target to which the templates will be aligned
		void setTargetCloud (TrackedCloud &target_cloud)
		{
			target_ = target_cloud;
			sac_ia_.setInputTarget (target_cloud.getTrackedCloud());
			sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures());
		}

		// Add the given cloud to the list of template clouds
		void setTemplateCloud (TrackedCloud &template_cloud)
		{
			template_ = template_cloud;
			sac_ia_.setInputCloud (template_cloud.getTrackedCloud());
			sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures());
		}

		// Align the given template cloud to the target specified by setTargetCloud ()
		AlignmentDetector::Result align ()
		{
			AlignmentDetector::Result result;
			pcl::PointCloud<pcl::PointXYZ> registration_output;
			sac_ia_.align (registration_output);

			result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
			result.final_transformation = sac_ia_.getFinalTransformation ();
			return result;
		}

		//// Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
		//void alignAll (std::vector<AlignmentDetector::Result, Eigen::aligned_allocator<Result> > &results)
		//{
		//  results.resize (templates_.size ());
		//  for (size_t i = 0; i < templates_.size (); ++i)
		//  {
		//	align (templates_[i], results[i]);
		//  }
		//}
	};
}