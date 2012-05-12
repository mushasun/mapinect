#ifndef I_POLYGON_H__
#define I_POLYGON_H__

#include "ofVec3f.h"
#include <vector>

namespace mapinect {

	class IPolygon {
		public:

			virtual const ofVec3f&			getVertex(int vertexNum) = 0;
			virtual int						getVertexCount() = 0;
			virtual const vector<ofVec3f>&	getVertexs() = 0;
			virtual const ofVec3f&			getNormal() = 0;

	};
}

#endif	// I_POLYGON_H__