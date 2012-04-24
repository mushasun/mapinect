#ifndef MAPINECT_PC_POLYHEDRON_H__
#define MAPINECT_PC_POLYHEDRON_H__

#include "PCModelObject.h"

#include "Polyhedron.h"
#include "PCPolygon.h"

namespace mapinect {

	class PCPolyhedron : public PCModelObject {
		public:
			PCPolyhedron(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ>::Ptr extendedCloud, int objId);
			
			virtual void		draw();
			virtual void		detectPrimitives();
			virtual void		applyTransformation();
			PCPolygon*			getPCPolygon(int index);
			int					getPCPolygonSize();
			virtual void		resetLod();
			virtual void		increaseLod();
			virtual void		addToModel(PointCloud<PointXYZ>::Ptr nuCloud);
			
		protected:
			//virtual PCPolygon*	createPCPolygon();

		private:
			void				updatePolygons();
			virtual void		unifyVertexs();
			bool				findBestFit(PCPolygon*, PCPolygon*& removed, bool& wasRemoved);
			vector<PCPolygon*>	detectPolygons(PointCloud<pcl::PointXYZ>::Ptr cloudTemp, float planeTolerance = 0.01, float pointsTolerance = 4.0, bool limitFaces = true);
			void				mergePolygons(vector<PCPolygon*> toMerge);
			vector<PCPolygon*>	discardPolygonsOutOfBox(vector<PCPolygon*> toDiscard);
			vector<PCPolygon*>	pcpolygons;

	};
}

#endif	// MAPINECT_PC_POLYHEDRON_H__