#ifndef MAPINECT_POLYHEDRON_H__
#define MAPINECT_POLYHEDRON_H__

#include "ModelObject.h"

#include "Polygon.h"
#include <vector>

namespace mapinect {
	class Polyhedron : public ModelObject {
		public:
			Polyhedron() { }
			virtual				~Polyhedron() { }

			void				addPolygon(Polygon*);
			Polygon*			getPolygonAt(int index);
			inline int			polygonCount()				{ return polygons.size(); }

			const ofVec3f&		getVertex(int index);

			virtual void		draw();
			
		private:
			vector<Polygon*>	polygons;
		
	};
}

#endif	// MAPINECT_POLYHEDRON_H__