#ifndef MAPINECT_TABLE_H__
#define MAPINECT_TABLE_H__

#include "PCQuadrilateral.h"

namespace mapinect {

	class Table : public PCQuadrilateral {
		public:
			Table(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud, int objId = -1)
				: PCQuadrilateral(coefficients, cloud, objId) { }

			inline bool detect()
			{
				return detectPolygon();
			}
	};
}

#endif	// MAPINECT_TABLE_H__