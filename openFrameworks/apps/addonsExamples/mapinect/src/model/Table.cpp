#include "Table.h"

#include "pointUtils.h"

namespace mapinect
{
	bool Table::isOnTable(const PCPtr& cloud)
	{
		//Busco el mayor y
		int idx_max = -1;
		float max = -1;
		for(int i = 0; i < cloud->points.size(); i++)
		{
			if(cloud->points.at(i).y > max)
			{
				idx_max = i;
				max = cloud->points.at(i).y;
			}
		}

		return abs(evaluatePoint(getCoefficients(),POINTXYZ_OFXVEC3F(cloud->points.at(idx_max)))) < 0.03;
	}

	bool Table::isParallelToTable(const PCPolygonPtr& polygon)
	{
		ofVec3f polygonNormal = polygon->getNormal();
		float dot = abs(getNormal().dot(polygonNormal));
		return dot > 0.9;
	}

}
