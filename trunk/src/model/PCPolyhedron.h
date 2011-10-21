#ifndef MAPINECT_PC_POLYHEDRON_H__
#define MAPINECT_PC_POLYHEDRON_H__

#include "PCModelObject.h"

#include "Polyhedron.h"
#include "PCPolygon.h"

namespace mapinect {

	class PCPolyhedron : public PCModelObject {
		public:
			PCPolyhedron(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ>::Ptr extendedCloud)
				: PCModelObject(cloud, extendedCloud) { }

			virtual void		draw();
			virtual void		detectPrimitives();

		protected:
			virtual PCPolygon*	createPCPolygon();

		private:
			vector<PCPolygon*>	pcpolygons;
	};
}

#endif	// MAPINECT_PC_POLYHEDRON_H__