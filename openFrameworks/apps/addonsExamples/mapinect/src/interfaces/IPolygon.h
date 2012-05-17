#ifndef I_POLYGON_H__
#define I_POLYGON_H__

#include "ofVec3f.h"
#include <vector>

namespace mapinect {

	typedef enum {
		kPolygonNameTop = 0,
		kPolygonNameSideA,
		kPolygonNameSideB,
		kPolygonNameSideC,
		kPolygonNameSideD,
		kPolygonNameBottom
	} IPolygonName;
	
	class IObject;

	class IPolygon {
		public:

			virtual int						getId() = 0;
			
			virtual const ofVec3f&			getCenter() = 0;
			virtual const ofVec3f&			getScale() = 0;
			virtual const ofVec3f&			getRotation() = 0;
			virtual const ofVec3f&			getNormal() = 0;

			virtual const IObject*			getContainer() = 0;
			virtual const IPolygonName&		getName() = 0;
			virtual const vector<ofVec3f>&	getVertexs() = 0;

	};
}

#endif	// I_POLYGON_H__