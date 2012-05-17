#ifndef I_OBJECT_H__
#define I_OBJECT_H__

#include "IPolygon.h"

namespace mapinect {

	class IObject {

		public:

			virtual int						getId() = 0;

			virtual const ofVec3f&			getCenter() = 0;
			virtual const ofVec3f&			getScale() = 0;
			virtual const ofVec3f&			getRotation() = 0;

			virtual const IPolygon*			getPolygon(const IPolygonName& polygonName) = 0;
			virtual const vector<IPolygon*>	getPolygons() = 0;

	};
}

#endif	// I_OBJECT_H__