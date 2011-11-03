#ifndef MAPINECT_PC_POLYGON_H__
#define MAPINECT_PC_POLYGON_H__

#include "PCModelObject.h"

#include "Polygon.h"

namespace mapinect {

	class PCPolygon : public PCModelObject {
		public:
			PCPolygon();
			virtual ~PCPolygon();

			Polygon*			getPolygonModelObject();
			
			virtual bool		detectPolygon(pcl::PointCloud<PointXYZ>::Ptr cloud, const std::vector<ofxVec3f>& vCloud);
			void				applyTransformation(Eigen::Affine3f* transformation);
			virtual void		increaseLodOfPolygon(PointCloud<PointXYZ>::Ptr nuCloud);
		protected:
			pcl::ModelCoefficients coefficients;
	};
}

#endif	// MAPINECT_PC_POLYGON_H__