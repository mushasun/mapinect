#include "PCModelObject.h"

#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation.h>

#include "Globals.h"
#include "ofGraphicsUtils.h"
#include "pointUtils.h"
#include "utils.h"


namespace mapinect {

	PCModelObject::PCModelObject(const PCPtr& cloud, int objId)
		: cloud(new PC(*cloud))
	{
		drawPointCloud = true;
		computeBoundingBox(cloud, vMin, vMax);
		transformation.setIdentity();
		if (objId == -1)	
		{
			static int objectId = 1;
			objId = objectId++;
		}
		setId(objId);
		lod = 1;

		this->setCenter(computeCentroid(cloud));

		this->setColor(ofColor(rand()%255,rand()%255,rand()%255).getHex());
	}

	PCModelObject::~PCModelObject() {

	}

	IObjectPtr PCModelObject::getMathModelApproximation() const
	{
		// This method should never be called. Implemented to avoid this class from being abstract
		assert(false);
		return IObjectPtr();
	}

	void PCModelObject::detectPrimitives() {

	}

	void PCModelObject::increaseLod(const PCPtr& nuCloud) {

	}

	void PCModelObject::resetLod() {
		lod = 1;
	}

	void PCModelObject::draw(){
		if (modelObject.get() != NULL) {
			modelObject->draw();
		}
		if (drawPointCloud) {
			static ofColor colors[] = { kRGBBlue, kRGBGreen, kRGBMagenta, kRGBCyan, kRGBYellow, kRGBPurple };
			ofSetColor(colors[getId() % 6]);
			ofVec3f w;
			glBegin(GL_POINTS);
			for (size_t i = 0; i < cloud->size(); i++) {
				ofVec3f v = POINTXYZ_OFXVEC3F(cloud->at(i));
				w = getScreenCoords(v);
				glVertex3f(w.x, w.y, 5);
			}
			glEnd();
			
		}
	}

	//void PCModelObject::updateCloud(const PCPtr& nuCloud) {
	//	cloud = nuCloud;
	//	lod++;
	//	increaseLod();
	//}

	void PCModelObject::applyTransformation (){
		PC transformed;
		transformPointCloud(*cloud, transformed, transformation);
		*cloud = transformed;
	}

	void PCModelObject::addToModel(const PCPtr& nuCloud){
		//DEBUG!!!!
		lod++;
		/////////

		//pcl::io::savePCDFileASCII ("pre.pcd", cloud);
		//pcl::io::savePCDFileASCII ("nu.pcd", *nuCloud);
		//applyTransformation();
		/*cloud += *nuCloud;*/
		//pcl::io::savePCDFileASCII ("transformed.pcd", cloud);
	}

	void PCModelObject::setAndUpdateCloud(const PCPtr& cloud)
	{
		setCloud(cloud);
		computeBoundingBox(cloud, vMin, vMax);
		
		this->setCenter(computeCentroid(cloud));

		transformation.setIdentity();
		lod = 1;
	}

	void PCModelObject::setDrawPointCloud(bool draw)		
	{ 
		drawPointCloud = draw; 
	}
}

