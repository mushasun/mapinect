#ifndef I_OBJECT_H__
#define I_OBJECT_H__

#include "IPolygon.h"
#include <boost/shared_ptr.hpp>

#define TABLE_ID		0

namespace mapinect
{
	class IObject
	{

		public:

			virtual int							getId() const = 0;

			virtual const ofVec3f&				getCenter() const = 0;
			virtual const ofVec3f&				getScale() const = 0;
			virtual const ofVec3f&				getRotation() const = 0;

			virtual const IPolygonPtr&			getPolygon(const IPolygonName& polygonName) const = 0;
			virtual const vector<IPolygonPtr>&	getPolygons() const = 0;

	};
}

#endif	// I_OBJECT_H__