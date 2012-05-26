#ifndef MAPINECT_PC_BOX_H__
#define MAPINECT_PC_BOX_H__

#include "PCPolyhedron.h"


namespace mapinect {

	class PCBox;

	typedef boost::shared_ptr<PCBox> PCBoxPtr;

	class PCBox : public PCPolyhedron{
		public:
			PCBox(const PCPtr& cloud, int objId);
			
			virtual void			detectPrimitives();
			virtual void			addToModel(const PCPtr& nuCloud);
			//BoxVertex&				getVertex(BoxVertexName v);

		private:
			virtual void					unifyVertexs();
			PCPolygonPtr					duplicatePol(const PCPolygonPtr& polygon,const vector<PCPolygonPtr>& newPolygons);
			virtual vector<PCPolygonPtr>	estimateHiddenPolygons(const vector<PCPolygonPtr>& newPolygons);

			virtual vector<PCPolygonPtr>	discardPolygonsOutOfBox(const vector<PCPolygonPtr>& toDiscard);

			list<IPolygonName>				getMissing(const vector<PCPolygonPtr>& estimated);
			PCPolygonPtr					getNextPolygon(IPolygonName toEstimate, const vector<PCPolygonPtr>& newPolygons);
			PCPolygonPtr					getPrevPolygon(IPolygonName toEstimate, const vector<PCPolygonPtr>& newPolygons);
			PCPolygonPtr					getOppositePolygon(IPolygonName toEstimate, const vector<PCPolygonPtr>& newPolygons);
			const PCPolygonPtr					getPCPolygon(IPolygonName name, const vector<PCPolygonPtr>& newPolygons);

			void							messureBox();

	};
}

#endif	// MAPINECT_PC_BOX_H__