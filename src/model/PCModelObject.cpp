#include "PCModelObject.h"

#include "pointUtils.h"

namespace mapinect {
	PCModelObject::PCModelObject() {
		modelObject = NULL;
		drawPointCloud = true;
	}

	PCModelObject::PCModelObject(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ>::Ptr extendedCloud) {
		drawPointCloud = true;
		modelObject = NULL;
		this->cloud = PointCloud<PointXYZ>(*cloud);
		this->extendedcloud = PointCloud<PointXYZ>(*extendedCloud);
		//PointCloud<pcl::PointXYZ>::Ptr cloudTemp (new PointCloud<PointXYZ>(*cloud));
		findPointCloudBoundingBox(cloud, vMin, vMax);
		transformation.setIdentity();
	}

	PCModelObject::~PCModelObject() {

	}

	void PCModelObject::detectPrimitives() {

	}

	void PCModelObject::draw() {
		if (modelObject != NULL) {
			modelObject->draw();
		}
		if (drawPointCloud) {
			glBegin(GL_POINTS);
			for (size_t i = 0; i < cloud.size(); i++) {
				PointXYZ p = cloud.at(i);
				glVertex3f(p.x, p.y, p.z);
			}
			glEnd();
		}
	}

	void PCModelObject::updateCloud(PointCloud<PointXYZ>::Ptr nuCloud) {
		cloud += (*nuCloud);
		detectPrimitives();
	}

}
