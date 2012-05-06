#include "PCPolygon.h"

#include <pcl/registration/transformation_estimation.h>
#include "ofVecUtils.h"

namespace mapinect {

	PCPolygon::PCPolygon(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud, int objId)
		: PCModelObject(cloud, objId), coefficients(coefficients)
	{
		modelObject = new Polygon();
		matched = NULL;
	}

	PCPolygon::~PCPolygon() {
		removeMatching();
		delete modelObject;
	}

	Polygon* PCPolygon::getPolygonModelObject() {
		return (Polygon*)modelObject;
	}

	ofVec3f PCPolygon::getNormal() const {
		ofVec3f normal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
		normal.normalize();
		return normal;
	}

	void PCPolygon::draw() {
		PCModelObject::draw();
		if (drawPointCloud) {
			if (getPolygonModelObject() != NULL) {
				ofSetColor(255,0,0);
				ofVec3f avg = average(getPolygonModelObject()->getVertexs());
				ofVec3f sAvg = gKinect->getScreenCoordsFromWorldCoords(avg);
				ofDrawBitmapString(ofToString(getId()), sAvg.x, sAvg.y);
			}
		}
	}

	bool PCPolygon::detectPolygon() {
		// No existing algorithm for a generic poligon detection.

		return false;
	}

	void PCPolygon::resetLod()
	{
		cloud->clear();
	}

	void PCPolygon::increaseLod(const PCPtr& nuCloud){
		// No existing algorithm for a generic poligon detection.
	}

	void PCPolygon::applyTransformation(Eigen::Affine3f* transformation)
	{
		pcl::transformPointCloud(*cloud, *cloud, *transformation);
		Eigen::Vector3f eVec;
		vector<ofVec3f> vertexs = getPolygonModelObject()->getVertexs();
		
		Eigen::Vector3f pointInPlane(0,0,-coefficients.values.at(3)/coefficients.values.at(2));//(0,0,-d/c)

		eVec.x() = coefficients.values.at(0);
		eVec.y() = coefficients.values.at(1);
		eVec.z() = coefficients.values.at(2);
		//eVec.w() = coefficients.values.at(3);

		eVec = (*transformation) * eVec;
		pointInPlane = (*transformation) * pointInPlane;

		coefficients.values.at(0) = eVec.x();
		coefficients.values.at(1) = eVec.y();
		coefficients.values.at(2) = eVec.z();
		coefficients.values.at(3) = - coefficients.values.at(0)*pointInPlane.x() //d = -ax0 -by0 -cz0
								    - coefficients.values.at(1)*pointInPlane.y()
									- coefficients.values.at(2)*pointInPlane.z();

		/*for(int i = 0; i < vertexs.size(); i++)
		{
			eVec.x() = vertexs.at(i).x;
			eVec.y() = vertexs.at(i).y;
			eVec.z() = vertexs.at(i).z;

			eVec = (*transformation) * eVec;

			vertexs.at(i).x = eVec.x();
			vertexs.at(i).y = eVec.y();
			vertexs.at(i).z = eVec.z();
		}*/
	}


	bool PCPolygon::matches(PCPolygon* polygon, PCPolygon*& removed, bool& wasRemoved)
	{
		bool result = true;
		wasRemoved = false;
		
		ofVec3f myNormal = getNormal();
		ofVec3f yourNormal = polygon->getNormal();
		float angle = acos(myNormal.dot(yourNormal));
		float estimator = fabsf(fabsf(angle - PI * 0.5) - PI * 0.5);
		const float NORMAL_TOLERANCE = PI / 4.0;
		if (estimator < NORMAL_TOLERANCE) {
			if (matched == NULL || estimator < matchedEstimator) {
				wasRemoved = matched != NULL;
				removed = matched;
				matched = polygon;
				matchedEstimator = estimator;
			}
		}
		else {
			result = false;
		}
		return result;
	}

	float distanceBetween(const ofVec3f& vA, const ofVec3f& vB) {
		return vA.distance(vB);
	}

	void PCPolygon::updateMatching() {
		if (hasMatching()) {
			vector<PairMatching> bestMatch = bestMatching(
				getPolygonModelObject()->getVertexs(),
				matched->getPolygonModelObject()->getVertexs(),
				distanceBetween);
			vector<ofVec3f> matchedVertexs(matched->getPolygonModelObject()->getVertexs());
			for (int i = 0; i < bestMatch.size(); i++) {
				getPolygonModelObject()->setVertex(bestMatch[i].ixA, matchedVertexs.at(bestMatch[i].ixB));
			}

			coefficients.header = matched->coefficients.header;
			coefficients.values = matched->coefficients.values;

			removeMatching();
		}
	}
	
	void PCPolygon::removeMatching() {
		if (hasMatching()) {
			delete matched;
			matched = NULL;
			matchedEstimator = MAX_FLOAT;
		}
	}

}
