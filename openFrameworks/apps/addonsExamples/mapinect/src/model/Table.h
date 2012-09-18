#ifndef MAPINECT_TABLE_H__
#define MAPINECT_TABLE_H__

#include "PCQuadrilateral.h"

namespace mapinect
{
	class Table : public PCQuadrilateral
	{
	public:
		static TablePtr create(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud);
		static TablePtr updateTablePlane(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud);
		static Eigen::Affine3f calculateRotations(const pcl::ModelCoefficients& coefficients);

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

		vector<ofVec3f> initOrderedVertexs;

		static void calibrateTable(TablePtr& table);

	};
}

#endif	// MAPINECT_TABLE_H__