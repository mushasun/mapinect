#include "transformationUtils.h"

#include <pcl/common/transforms.h>

#include "Globals.h"
#include "pointUtils.h"
#include "Transformation.h"

vector<ofVec3f>	eigenVectorToOfVecVector(const vector<Eigen::Vector3f>& v)
{
	vector<ofVec3f> vec;
	for(int i = 0; i < v.size(); i++)
	{
		Eigen::Vector3f ev = v.at(i);
		vec.push_back(ofVec3f(ev.x(), ev.y(), ev.z()));
	}
	return vec;
}

Eigen::Affine3f		getTranslationMatrix(const ofVec3f& v)
{
	Eigen::Affine3f result;
	result = Eigen::Translation<float, 3>(v.x, v.y, v.z);
	return result;
}

Eigen::Affine3f		getScaleMatrix(const ofVec3f& v)
{
	Eigen::Affine3f result;
	result = Eigen::Scaling<float>(v.x, v.y, v.z);
	return result;
}

Eigen::Affine3f		getScaleMatrix(float scale)
{
	Eigen::Affine3f result;
	result = Eigen::Scaling<float>(scale, scale, scale);
	return result;
}

Eigen::Affine3f		getRotationMatrix(const ofVec3f& axis, float angleRad)
{
	Eigen::Affine3f result;
	result = Eigen::AngleAxis<float>(angleRad, OFVEC3F_EIGEN3F(axis));
	return result;
}

PCXYZ eyePos()
{
	return transformPoint(PCXYZ(0, 0, 0),  gTransformation->getWorldTransformation());
}

PCXYZ transformPoint(const PCXYZ& p, const Eigen::Affine3f& transform)
{
	return pcl::transformPoint(p, transform);
}

ofVec3f transformPoint(const ofVec3f& v, const Eigen::Affine3f& transform)
{
	PCXYZ transformedPoint(transformPoint(OFVEC3F_PCXYZ(v), transform));
	return PCXYZ_OFVEC3F(transformedPoint);
}

PCPtr transformCloud(const PCPtr& cloud, const Eigen::Affine3f& transform)
{
	PCPtr transformedCloud(new PC());
	pcl::transformPointCloud(*cloud, *transformedCloud, transform);
	return transformedCloud;
}

vector<ofVec3f> transformVector(const vector<ofVec3f>& v, const Eigen::Affine3f& transform)
{
	vector<ofVec3f> result;

	for (int i = 0; i < v.size(); i++)
		result.push_back(transformPoint(v[i], transform));
	
	return result;
}

mapinect::Polygon3D transformPolygon3D(const mapinect::Polygon3D& polygon, const Eigen::Affine3f& transform)
{
	mapinect::Polygon3D pol(polygon);
	vector<ofVec3f> vertexs = pol.getVertexs();
	vector<ofVec3f> transVertexs;

	for(int i = 0; i < vertexs.size(); i++)
	{
		transVertexs.push_back(transformPoint(vertexs.at(i),transform));
	}

	pol.setVertexs(transVertexs);
	return pol;
}

PCPtr getScreenCoords(const PCPtr& transformedWorldCloud)
{
	PCPtr screenCloud(new PC());
	for (PC::const_iterator p = transformedWorldCloud->begin(); p != transformedWorldCloud->end(); ++p)
	{
		screenCloud->push_back(getScreenCoords(*p));
	}
	return screenCloud;
}

vector<ofVec3f> getScreenCoords(const vector<ofVec3f>& transformedWorldCloud)
{
	vector<ofVec3f> screenPoints;
	for (vector<ofVec3f>::const_iterator v = transformedWorldCloud.begin(); v != transformedWorldCloud.end(); ++v)
	{
		screenPoints.push_back(getScreenCoords(*v));
	}
	return screenPoints;
}

PCXYZ getScreenCoords(const PCXYZ& transformedWorldPoint)
{
	// Apply to world point the inverse transformation
	PCXYZ transf = pcl::transformPoint(transformedWorldPoint, gTransformation->getInverseWorldTransformation());
	// Then, we call the depth device instance to transform to the depth image coordinates
	ofVec3f screenCoord = gKinect->getScreenCoordsFromWorldCoords(PCXYZ_OFVEC3F(transf));
	return OFVEC3F_PCXYZ(screenCoord);
}

ofVec3f getScreenCoords(const ofVec3f& transformedWorldPoint)
{
	PCXYZ screenPoint(getScreenCoords(OFVEC3F_PCXYZ(transformedWorldPoint)));
	return PCXYZ_OFVEC3F(screenPoint);
}
