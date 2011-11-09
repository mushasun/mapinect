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

			ofxVec3f			getNormal();
			
			virtual void		draw();

			virtual bool		detectPolygon(pcl::PointCloud<PointXYZ>::Ptr cloud, const std::vector<ofxVec3f>& vCloud);
			void				applyTransformation(Eigen::Affine3f* transformation);
			virtual void		resetLod();
			virtual void		increaseLod(PointCloud<PointXYZ>::Ptr nuCloud);

			inline bool			hasMatching()		{ return matched != NULL; }
			virtual bool		matches(PCPolygon* polygon, PCPolygon*& removed, bool& wasRemoved);
			void				updateMatching();
			void				removeMatching();

		protected:
			pcl::ModelCoefficients coefficients;

			PCPolygon*			matched;
			float				matchedEstimator;

	};
}

#endif	// MAPINECT_PC_POLYGON_H__