#include "PCModelObject.h"

#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation.h>

#include "Globals.h"
#include "ofGraphicsUtils.h"
#include "pointUtils.h"
#include "transformationUtils.h"
#include "utils.h"


namespace mapinect {

	PCModelObject::PCModelObject(const PCPtr& cloud, int objId)
	{
		drawPointCloud = true;
		setCloud(cloud);
		transformation.setIdentity();
		if (objId == -1)	
		{
			static int objectId = 1;
			objId = objectId++;
		}
		setId(objId);
		lod = 1;

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
			//glPointSize(2.0);
			glBegin(GL_POINTS);
			for (PC::const_iterator p = cloudScreenCoords->begin(); p != cloudScreenCoords->end(); ++p) {
				glVertex3f(p->x, p->y, 5);
			}
			glEnd();
			
		}
	}

	void PCModelObject::addToModel(const PCPtr& nuCloud){
		//DEBUG!!!!
		lod++;
		/////////

		//pcl::io::savePCDFileASCII ("pre.pcd", cloud);
		//pcl::io::savePCDFileASCII ("nu.pcd", *nuCloud);
		/*cloud += *nuCloud;*/
		//pcl::io::savePCDFileASCII ("transformed.pcd", cloud);
	}

	void PCModelObject::setDrawPointCloud(bool draw)		
	{ 
		drawPointCloud = draw;
	}

	void PCModelObject::setCloud(const PCPtr& nuCloud)
	{
		cloud = nuCloud;
		computeBoundingBox(cloud, vMin, vMax);
		setCenter(computeCentroid(cloud));
		cloudScreenCoords = getScreenCoords(cloud);
	}

}

