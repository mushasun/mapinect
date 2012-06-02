#include "DataObject.h"

namespace mapinect
{
	const IPolygonPtr& DataObject::getPolygon(const IPolygonName& name) const
	{
		for (vector<IPolygonPtr>::const_iterator p = polygons.begin(); p != polygons.end(); ++p)
			if ((*p)->getName() == name)
				return *p;
		return IPolygonPtr();
	}
}
