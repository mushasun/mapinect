#include "Table.h"

#include "Constants.h"
#include "DataObject.h"
#include "Feature.h"
#include "Globals.h"
#include "mapinectTypes.h"
#include "pointUtils.h"

namespace mapinect
{
	TablePtr Table::Create(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud)
	{
		PCPtr transformedCloud(cloud);
		pcl::ModelCoefficients transformedCoefficients(coefficients);
		
		if (!IsFeatureMoveArmActive())
		{
			// We want to set the table as the plane y = 0
			// While we don't have the arm transformation, we will translate the origin to
			// table's centroid directly from Kinect's origin
			Eigen::Vector3f axisX(1, 0, 0);
			Eigen::Affine3f rotationX;
			ofVec3f newYaxis = -(::getNormal(coefficients).getNormalized());
			float angle = newYaxis.angleRad(ofVec3f(0, 1, 0));
			rotationX = Eigen::AngleAxis<float>(-angle, axisX);

			Eigen::Affine3f translation;
			ofVec3f centroid(computeCentroid(cloud));
			translation = Eigen::Translation<float, 3>(-centroid.x, -centroid.y, -centroid.z);

			Eigen::Affine3f composedMatrix = rotationX * translation;
		
			gTransformation->setWorldTransformation(composedMatrix);

			transformedCloud = PCPtr(transformCloud(cloud, composedMatrix));

			transformedCoefficients.values[0] = 0;
			transformedCoefficients.values[1] = -1;
			transformedCoefficients.values[2] = 0;
			transformedCoefficients.values[3] = 0;
		}
		
		TablePtr table(new Table(transformedCoefficients, transformedCloud));
		table->detect();
		table->setDrawPointCloud(false);
		gModel->setTable(table);

		return table;
	}

	IObjectPtr Table::getMathModelApproximation() const
	{
		vector<IPolygonPtr> polygons;
		polygons.push_back(getMathPolygonModelApproximation());
		IObjectPtr result(new DataObject(getId(), getCenter(), getScale(), getRotation(), polygons));
		dynamic_cast<Polygon*>(polygons[0].get())->setContainer(result);
		return result;
	}

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

		if (idx_max >= 0)
		{
			return abs(evaluatePoint(getCoefficients(), PCXYZ_OFVEC3F(cloud->points.at(idx_max))))
				< Constants::TABLE_HEIGHT_TOLERANCE() * 2.0f;
		}
		else
		{
			return false;
		}
	}

	bool Table::isParallelToTable(const PCPolygonPtr& polygon)
	{
		ofVec3f polygonNormal = polygon->getNormal();
		float dot = abs(getNormal().dot(polygonNormal));
		return dot > 0.9;
	}

	bool Table::isOverTable(const PCPtr& cloud)
	{
		Polygon3D pol = this->getMathPolygonModelApproximation()->getMathModel();

		for(int i = 0; i < cloud->size(); i++)
		{
			if (pol.isInPolygon(pol.getPlane().project(PCXYZ_OFVEC3F(cloud->at(i)))))
				return true;
		}
		return false;

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
