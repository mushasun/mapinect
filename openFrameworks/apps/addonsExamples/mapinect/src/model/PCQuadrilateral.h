#ifndef MAPINECT_PC_QUADRILATERAL_H__
#define MAPINECT_PC_QUADRILATERAL_H__

#include "PCPolygon.h"

#include "ofVec3f.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <vector>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include "pointUtils.h"

namespace mapinect {

	class PCQuadrilateral : public PCPolygon {
		public:
			PCQuadrilateral(pcl::ModelCoefficients coefficients);

			virtual bool							detectPolygon(pcl::PointCloud<PointXYZ>::Ptr cloud,
														const std::vector<ofVec3f>& vCloud);

			inline const pcl::PointIndices::Ptr		getVertexIdxs() { return vertexIdxs; }
			virtual void							increaseLod(PointCloud<PointXYZ>::Ptr nuCloud);
		private:
			pcl::PointIndices::Ptr					vertexIdxs;
	};
}

#endif	// MAPINECT_PC_QUADRILATERAL_H__