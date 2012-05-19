#ifndef MAPINECT_PC_POLYHEDRON_H__
#define MAPINECT_PC_POLYHEDRON_H__

#include "PCModelObject.h"
#include "IObject.h"

#include "Polyhedron.h"
#include "PCPolygon.h"

namespace mapinect {

	class PCPolyhedron;

	typedef boost::shared_ptr<PCPolyhedron> PCPolyhedronPtr;

	class PCPolyhedron : public PCModelObject, public IObject {
		public:
			PCPolyhedron(const PCPtr& cloud, int objId);
			
			virtual void			draw();
			virtual void			detectPrimitives();
			virtual void			applyTransformation();
			const PCPolygonPtr&		getPCPolygon(int index);
			int						getPCPolygonSize();
			virtual void			resetLod();
			virtual void			increaseLod();
			virtual void			addToModel(const PCPtr& nuCloud);
			virtual void			setAndUpdateCloud(const PCPtr& cloud);

			inline int						getId()							{ return PCModelObject::getId(); }

			inline const ofVec3f&			getCenter()						{ return PCModelObject::getCenter(); }
			inline const ofVec3f&			getScale()						{ return PCModelObject::getScale(); }
			inline const ofVec3f&			getRotation()					{ return PCModelObject::getRotation(); }

			const IPolygon*					getPolygon(const IPolygonName&);
			inline const vector<IPolygon*>	getPolygons()					{ return polygonsCache; }
			
			vector<PCPolygonPtr>	PCPolyhedron::estimateHiddenPolygons(const vector<PCPolygonPtr>& newPolygons);
		private:
			void					updatePolygons();
			virtual void			unifyVertexs();
			bool					findBestFit(const PCPolygonPtr&, PCPolygonPtr& removed, bool& wasRemoved);
			vector<PCPolygonPtr>	detectPolygons(const PCPtr& cloudTemp, float planeTolerance = 0.01, float pointsTolerance = 4.0, bool limitFaces = true);
			void					mergePolygons(vector<PCPolygonPtr>& toMerge);
			PCPolygonPtr			duplicatePol(const PCPolygonPtr& polygon,const vector<PCPolygonPtr>& newPolygons);

			vector<PCPolygonPtr>	discardPolygonsOutOfBox(const vector<PCPolygonPtr>& toDiscard);

			vector<PCPolygonPtr>	pcpolygons;
			vector<IPolygon*>		polygonsCache;
			PCPtr					getCurrentVertex();
			

	};
}

#endif	// MAPINECT_PC_POLYHEDRON_H__