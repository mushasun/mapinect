#ifndef MAPINECT_PC_QUADRILATERAL_H__
#define MAPINECT_PC_QUADRILATERAL_H__

#include "PCPolygon.h"

#include "ofxVec3f.h"
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

			virtual bool							detectPolygon(const std::vector<ofxVec3f>& cloud);
			virtual bool							detectPolygon2(const std::vector<ofxVec3f>& cloud);

			inline const pcl::PointIndices::Ptr		getVertexIdxs() { return vertexIdxs; }
			virtual void							increaseLodOfPolygon(PointCloud<PointXYZ>::Ptr nuCloud);
		private:
			pcl::PointIndices::Ptr					vertexIdxs;
	};
}

#endif	// MAPINECT_PC_QUADRILATERAL_H__