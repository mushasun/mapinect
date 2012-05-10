#include "PCModelObject.h"

#include <pcl/registration/transformation_estimation.h>
#include <pcl/io/pcd_io.h>

#include "Globals.h"
#include "ofGraphicsUtils.h"
#include "pointUtils.h"
#include "utils.h"


namespace mapinect {

	PCModelObject::PCModelObject(const PCPtr& cloud, int objId)
		: cloud(new PC(*cloud))
	{
		drawPointCloud = true;
		findPointCloudBoundingBox(cloud, vMin, vMax);
		transformation.setIdentity();
		if (objId == -1)
		{
			static int objectId = 0;
			objId = objectId++;
		}
		setId(objId);
		lod = 1;
	}

	PCModelObject::~PCModelObject() {

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
			ofSetHexColor(colors[getId() % 6]);
			ofVec3f w;
			glBegin(GL_POINTS);
			for (size_t i = 0; i < cloud->size(); i++) {
				ofVec3f v = POINTXYZ_OFXVEC3F(cloud->at(i));
				w = gKinect->getScreenCoordsFromWorldCoords(v);
				glVertex3f(w.x, w.y, 5);
			}
			glEnd();
			
		}
	}

	void PCModelObject::updateCloud(const PCPtr& nuCloud) {
		cloud = nuCloud;
		lod++;
		increaseLod();
	}

	void PCModelObject::applyTransformation (){
		PC transformed;
		transformPointCloud(*cloud, transformed, transformation);
		*cloud = transformed;
	}

	void PCModelObject::addToModel(const PCPtr& nuCloud){
		//pcl::io::savePCDFileASCII ("pre.pcd", cloud);
		//pcl::io::savePCDFileASCII ("nu.pcd", *nuCloud);
		//applyTransformation();
		/*cloud += *nuCloud;*/
		//pcl::io::savePCDFileASCII ("transformed.pcd", cloud);
	}
}

