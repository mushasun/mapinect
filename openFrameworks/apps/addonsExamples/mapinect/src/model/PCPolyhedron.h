#ifndef MAPINECT_PC_POLYHEDRON_H__
#define MAPINECT_PC_POLYHEDRON_H__

#include "PCModelObject.h"
#include "IObject.h"

#include "PCPolygon.h"
#include <list>

#include "ofxMutex.h"

namespace mapinect {

	class PCPolyhedron;

	typedef boost::shared_ptr<PCPolyhedron> PCPolyhedronPtr;

	class PCPolyhedron : public PCModelObject {
		public:
			PCPolyhedron(const PCPtr& cloud, int objId);
			
			virtual IObjectPtr		getMathModelApproximation() const;

			virtual void			draw();
			virtual void			detectPrimitives();
			virtual void			resetLod();
			virtual void			increaseLod(const PCPtr& nuCloud);
			virtual void			addToModel(const PCPtr& nuCloud);

			inline const vector<IPolygon*>	getPolygons()						{ return polygonsCache; }
			const vector<IPolygonPtr>		getVisiblePolygons();
			inline const vector<ofVec3f>	getVertexs()						{ return vertexs; }
			inline void						resetOccludedFaces()				{ occludedFaces.clear(); }
			inline void						setOccludedFace(IPolygonName name)	{ occludedFaces.push_back(name); }
			bool							isFaceOccluded(IPolygonName name);
		private:
			vector<PCPolygonPtr>			updatePolygons();
			PCPolygonPtr					findCloserPolygon(PCPolygonPtr pol, vector<PCPolygonPtr> polygons);
			bool							findBestFit(const PCPolygonPtr&, PCPolygonPtr& removed, bool& wasRemoved);
			vector<PCPolygonPtr>			detectPolygons(const PCPtr& cloudTemp, float planeTolerance = 0.003, float pointsTolerance = 4.0, bool limitFaces = true);
			virtual vector<PCPolygonPtr>	estimateHiddenPolygons(const vector<PCPolygonPtr>& newPolygons, bool& estimationOk);
			virtual vector<PCPolygonPtr>	discardPolygonsOutOfBox(const vector<PCPolygonPtr>& toDiscard);
			virtual vector<PCPolygonPtr>	discardPolygonsOutOfBox(const vector<PCPolygonPtr>& toDiscard, const vector<PCPolygonPtr>& inPolygon);
			virtual void					namePolygons(vector<PCPolygonPtr>& toName);
			
			vector<IPolygonName>			occludedFaces;

		protected:
			vector<PCPolygonPtr>			pcpolygons;
			vector<IPolygon*>				polygonsCache;
			vector<ofVec3f>					vertexs;
			bool							fullEstimation;

			virtual void					unifyVertexs();
			virtual vector<PCPolygonPtr>	mergePolygons(vector<PCPolygonPtr>& toMerge);
			const PCPolygonPtr&				getPCPolygon(int index);
			int								getPCPolygonSize();
			virtual bool					validate();
			virtual float					getVolume() = 0;

			mutable ofxMutex				pcPolygonsMutex;

	};
}

#endif	// MAPINECT_PC_POLYHEDRON_H__