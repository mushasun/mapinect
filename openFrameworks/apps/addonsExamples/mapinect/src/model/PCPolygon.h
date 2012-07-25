#ifndef MAPINECT_PC_POLYGON_H__
#define MAPINECT_PC_POLYGON_H__

#include "PCModelObject.h"

#include <pcl/ModelCoefficients.h>
#include "Polygon.h"

namespace mapinect {

	class PCPolygon;

	typedef boost::shared_ptr<PCPolygon> PCPolygonPtr;

	class PCPolygon : public PCModelObject {
		public:
			PCPolygon(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud, int objId = -1, bool estimated = false);
			virtual ~PCPolygon();

			IPolygonPtr			getMathPolygonModelApproximation() const;

			const Polygon*		getPolygonModelObject() const;
			Polygon*			getPolygonModelObject();

			ofVec3f				getNormal();
			
			virtual void		draw();

			virtual bool		detectPolygon();
			virtual void		resetLod();
			virtual void		increaseLod(const PCPtr& nuCloud);

			inline bool			hasMatching()					{ return matched != NULL; }
			virtual bool		matches(const PCPolygonPtr& polygon, PCPolygonPtr& removed, bool& wasRemoved);
			void				updateMatching();
			void				removeMatching();
			void				rollBackMatching();
			inline pcl::ModelCoefficients getCoefficients()		{ return coefficients; }
			inline void			setEstimated(bool value) { estimated = value; }
			inline bool			isEstimated() { return estimated; }
			Eigen::Affine3f		getMatchingTransformation();
		protected:
			pcl::ModelCoefficients coefficients;

			PCPolygonPtr		matched;
			float				matchedEstimator;
			float				matchedDistance;
			bool				estimated;
			Eigen::Affine3f		matchingTransformation;
		private:
			vector<ofVec3f>		rollBackVertexs;
			Plane3D				rollBackPlane;
			pcl::ModelCoefficients	rollBackCoefficients;
			PCPtr				rollBackCloud;
	};
}

#endif	// MAPINECT_PC_POLYGON_H__