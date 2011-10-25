#include "PCModelObject.h"
#include "pointUtils.h"
#include "utils.h"
#include <pcl/registration/transformation_estimation.h>

namespace mapinect {
	PCModelObject::PCModelObject() {
		modelObject = NULL;
		drawPointCloud = true;
	}

	PCModelObject::PCModelObject(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ>::Ptr extendedCloud)
	{
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
	PCModelObject::PCModelObject(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ>::Ptr extendedCloud, int objId)
	{
		drawPointCloud = true;
		modelObject = NULL;
		this->cloud = PointCloud<PointXYZ>(*cloud);
		this->extendedcloud = PointCloud<PointXYZ>(*extendedCloud);
		//PointCloud<pcl::PointXYZ>::Ptr cloudTemp (new PointCloud<PointXYZ>(*cloud));
		findPointCloudBoundingBox(cloud, vMin, vMax);
		transformation.setIdentity();
		id = objId;
	}

	void PCModelObject::detectPrimitives() {

	}

	void PCModelObject::draw(){
		/*if (modelObject != NULL) {
			modelObject->draw();
		}*/
		if (drawPointCloud) {
			ofSetColor(255,0,0);
			glBegin(GL_POINTS);
			for (size_t i = 0; i < cloud.size(); i++) {
				ofxVec3f v = POINTXYZ_OFXVEC3F(cloud.at(i));
				ofxVec3f w = gKinect->getScreenCoordsFromWorldCoords(v);
				glVertex3f(w.x, w.y, 5);
			}
			glEnd();
		}
	}

	void PCModelObject::updateCloud(PointCloud<PointXYZ>::Ptr nuCloud) {
		cloud += (*nuCloud);
		detectPrimitives();
	}

	void PCModelObject::applyTransformation (){
		PointCloud<PointXYZ> transformed;
		transformPointCloud(cloud,transformed,transformation);
		cloud = transformed;
	}
}

