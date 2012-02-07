#ifndef MAPINECT_ALIGNMENT_H__
#define MAPINECT_ALIGNMENT_H__
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>

using namespace pcl;

class Alignment
{
	public:
		Alignment();
		void Align();
};

#endif	// MAPINECT_ALIGNMENT_H__