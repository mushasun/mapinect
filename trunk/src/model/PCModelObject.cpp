#include "PCModelObject.h"
#include "pointUtils.h"
#include "utils.h"
#include <pcl/registration/transformation_estimation.h>
#include <pcl/io/pcd_io.h>


namespace mapinect {
	PCModelObject::PCModelObject() {
		modelObject = NULL;
		drawPointCloud = true;
		lod = 1;
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
		lod = 1;
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
		setId(objId);
		lod = 1;
	}

	void PCModelObject::detectPrimitives() {

	}

	void PCModelObject::increaseLod() {

	}

	void PCModelObject::resetLod() {
		lod = 1;
	}

	void PCModelObject::draw(){
		if (modelObject != NULL) {
			modelObject->draw();
		}
		if (drawPointCloud) {
			static int colors[] = { kRGBBlue, kRGBGreen, kRGBRed, 0xFFFF00, 0xFF00FF, 0x00FFFF };
			ofSetColor(colors[getId() % 6]);
			ofxVec3f w;
			glBegin(GL_POINTS);
			for (size_t i = 0; i < cloud.size(); i++) {
				ofxVec3f v = POINTXYZ_OFXVEC3F(cloud.at(i));
				w = gKinect->getScreenCoordsFromWorldCoords(v);
				glVertex3f(w.x, w.y, 5);
			}
			glEnd();
			
		}
	}

	void PCModelObject::updateCloud(PointCloud<PointXYZ>::Ptr nuCloud) {
		cloud = *nuCloud;
		lod++;
		increaseLod();
	}

	void PCModelObject::applyTransformation (){
		PointCloud<PointXYZ> transformed;
		transformPointCloud(cloud,transformed,transformation);
		cloud = transformed;
	}

	void PCModelObject::addToModel(PointCloud<PointXYZ>::Ptr nuCloud){
		//pcl::io::savePCDFileASCII ("pre.pcd", cloud);
		//pcl::io::savePCDFileASCII ("nu.pcd", *nuCloud);
		//applyTransformation();
		/*cloud += *nuCloud;*/
		//pcl::io::savePCDFileASCII ("transformed.pcd", cloud);
	}
}

