#ifndef MAPINECT_TABLE_H__
#define MAPINECT_TABLE_H__

#include "PCQuadrilateral.h"

namespace mapinect {

	class Table : public PCQuadrilateral {
	public:
		static TablePtr Create(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud);
		void SetTablePlane(const pcl::ModelCoefficients& coefficients);

		virtual IObjectPtr		getMathModelApproximation() const;

		inline bool detect()
		{
			return detectPolygon();
		}

		bool isOnTable(const PCPtr& cloud);
		bool isOverTable(const PCPtr& cloud);
		bool isParallelToTable(const mapinect::PCPolygonPtr& polygon);

	private:
		Table(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud)
			: PCQuadrilateral(coefficients, cloud, TABLE_ID) { }
	};
}

#endif	// MAPINECT_TABLE_H__