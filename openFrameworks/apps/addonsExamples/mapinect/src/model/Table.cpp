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

	bool Table::isOverTable(const PCPtr& cloud)
	{
		ofVec3f tableNorm = this->getNormal();
	
		ofVec3f minV, maxV;
		minV = this->getvMin();
		maxV = this->getvMax();

		maxV += tableNorm;
	
		/*createCloud(maxV,"maxt.pcd");
		createCloud(minV,"mint.pcd");
		pcl::io::savePCDFile("table.pcd",*table->getCloud());
		pcl::io::savePCDFile("hand.pcd",*cloud);*/
		//Alcanza con que un punto esté sobre la mesa
	
		for(int i = 0; i < cloud->size(); i++)
		{
			pcl::PointXYZ pto = cloud->at(i);
			if((pto.x > min(minV.x,maxV.x) && pto.x < max(maxV.x,minV.x)) &&
			   (pto.y > min(minV.y,maxV.y) && pto.y < max(maxV.y,minV.y)) &&
			   (pto.z > min(minV.z,maxV.z) && pto.z < max(maxV.z,minV.z)))
			   return true;
		}
	
		return false;

	}
}
