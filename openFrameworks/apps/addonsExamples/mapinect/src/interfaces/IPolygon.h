#ifndef I_POLYGON_H__
#define I_POLYGON_H__

#include <vector>
#include <boost/shared_ptr.hpp>

#include "ofVec3f.h"
#include "Polygon3D.h"

namespace mapinect
{
	typedef enum
	{
		kPolygonNameUnknown = -1,
		kPolygonNameTop = 0,
		kPolygonNameSideA,
		kPolygonNameSideB,
		kPolygonNameSideC,
		kPolygonNameSideD,
		kPolygonNameBottom
	} IPolygonName;
	
	class IPolygon;
	typedef boost::shared_ptr<IPolygon> IPolygonPtr;

	class IObject;
	typedef boost::shared_ptr<IObject> IObjectPtr;

	class IPolygon
	{
		public:

			virtual int						getId() const = 0;
			virtual const Polygon3D&		getMathModel() const = 0;
			virtual const IObjectPtr&		getContainer() const = 0;
			virtual const IPolygonName&		getName() const = 0;

			virtual int						getBestOriginVertexIndex() const = 0;
	};
}

#endif	// I_POLYGON_H__