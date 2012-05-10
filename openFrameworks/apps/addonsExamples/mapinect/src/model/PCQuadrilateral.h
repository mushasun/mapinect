#ifndef MAPINECT_PC_QUADRILATERAL_H__
#define MAPINECT_PC_QUADRILATERAL_H__

#include "PCPolygon.h"

#include <pcl/PointIndices.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/point_types.h>
//#include <vector>
//#include <pcl/registration/transformation_estimation.h>
//#include <pcl/registration/transformation_estimation_svd.h>
//#include "pointUtils.h"

namespace mapinect {

	class PCQuadrilateral;

	typedef boost::shared_ptr<PCQuadrilateral> PCQuadrilateralPtr;

	class PCQuadrilateral : public PCPolygon {
		public:
			PCQuadrilateral(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud, int objId = -1)
				: PCPolygon(coefficients, cloud, objId) { }

			virtual bool							detectPolygon();

			inline const pcl::PointIndices::Ptr		getVertexIdxs() { return vertexIdxs; }
			virtual void							increaseLod(const PCPtr& nuCloud);
		private:
			pcl::PointIndices::Ptr					vertexIdxs;
	};
}

#endif	// MAPINECT_PC_QUADRILATERAL_H__