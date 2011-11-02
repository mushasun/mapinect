#include "PCPolygon.h"
#include <pcl/registration/transformation_estimation.h>

namespace mapinect {

	PCPolygon::PCPolygon() {
		modelObject = new Polygon();
	}

	PCPolygon::~PCPolygon() {
		delete modelObject;
	}

	Polygon* PCPolygon::getPolygonModelObject() {
		return (Polygon*)modelObject;
	}

	bool PCPolygon::detectPolygon(const std::vector<ofxVec3f>& vCloud) {
		// No existing algorithm for a generic poligon detection.

		return false;
	}

	void PCPolygon::increaseLodOfPolygon(PointCloud<PointXYZ>::Ptr nuCloud){
		// No existing algorithm for a generic poligon detection.
	}

	void PCPolygon::applyTransformation(Eigen::Affine3f* transformation)
	{
		pcl::transformPointCloud(cloud,cloud,*transformation);
	}

}
