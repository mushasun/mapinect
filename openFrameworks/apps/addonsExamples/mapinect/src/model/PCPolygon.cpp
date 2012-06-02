#include "PCPolygon.h"

#include <pcl/registration/transformation_estimation.h>
#include <pcl/features/normal_3d.h>

#include "ofGraphicsUtils.h"

#include "Globals.h"
#include "ofVecUtils.h"
#include "utils.h"
#include "pointUtils.h"
#include "Plane3D.h"


namespace mapinect {

	PCPolygon::PCPolygon(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud, int objId, bool estimated)
		: PCModelObject(cloud, objId)
	{
		// Point the normal towards the viewpoint
		this->coefficients = coefficients;
		this->estimated = estimated;
		if(!estimated)
		{
			pcl::flipNormalTowardsViewpoint(cloud->at(0),
										0,0,0,
										this->coefficients.values[0], this->coefficients.values[1], this->coefficients.values[2]);
			if(this->coefficients.values != coefficients.values)
			{
				Plane3D plane(POINTXYZ_OFXVEC3F(cloud->at(0)),
							  ofVec3f(this->coefficients.values[0], this->coefficients.values[1], this->coefficients.values[2]));
				this->coefficients = plane.getCoefficients();
			}
		}
		matchedDistance = numeric_limits<float>::max();
		modelObject = ModelObjectPtr(new Polygon(coefficients));
	}

	PCPolygon::~PCPolygon() {
		removeMatching();
	}

	IPolygonPtr PCPolygon::getMathPolygonModelApproximation() const
	{
		IPolygonPtr result(getPolygonModelObject()->clone());
		return result;
	}

	const Polygon* PCPolygon::getPolygonModelObject() const
	{
		return (const Polygon*)modelObject.get();
	}

	Polygon* PCPolygon::getPolygonModelObject()
	{
		return (Polygon*)modelObject.get();
	}

	ofVec3f PCPolygon::getNormal() {
		ofVec3f normal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
		normal.normalize();
		return normal;
	}

	void PCPolygon::draw() {
		PCModelObject::draw();
		if (drawPointCloud) {
			if (getPolygonModelObject() != NULL) {
				ofSetColor(255,0,0);
				ofVec3f avg = computeCentroid(getPolygonModelObject()->getMathModel().getVertexs());
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
		//cloud->clear();
	}

	void PCPolygon::increaseLod(const PCPtr& nuCloud){
		// No existing algorithm for a generic poligon detection.
	}

	void PCPolygon::applyTransformation(Eigen::Affine3f* transformation)
	{
		saveCloudAsFile("preTransform.pcd",*cloud);
		pcl::transformPointCloud(*cloud, *cloud, *transformation);
		saveCloudAsFile("postTransform.pcd",*cloud);

		Eigen::Vector3f eVec(coefficients.values.at(0),coefficients.values.at(1),coefficients.values.at(2));
		vector<ofVec3f> vertexs = getPolygonModelObject()->getMathModel().getVertexs();
		
		Eigen::Vector3f pointInPlane(0,0,-coefficients.values.at(3)/coefficients.values.at(2));//(0,0,-d/c)

		eVec = (*transformation) * eVec;
		pointInPlane = (*transformation) * pointInPlane;

		coefficients.values.at(0) = eVec.x();
		coefficients.values.at(1) = eVec.y();
		coefficients.values.at(2) = eVec.z();
		coefficients.values.at(3) = - coefficients.values.at(0)*pointInPlane.x() //d = -ax0 -by0 -cz0
								    - coefficients.values.at(1)*pointInPlane.y()
									- coefficients.values.at(2)*pointInPlane.z();

		detectPolygon();
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


	bool PCPolygon::matches(const PCPolygonPtr& polygon, PCPolygonPtr& removed, bool& wasRemoved)
	{
		bool result = true;
		wasRemoved = false;
		
		ofVec3f myNormal = getNormal();
		ofVec3f yourNormal = polygon->getNormal();
		ofVec3f axis;
		{
			ofxScopedMutex osm(gModel->tableMutex);
			axis = gModel->getTable()->getNormal();
		}
		float angle = acos(myNormal.dot(yourNormal));
		float estimator = fabsf(fabsf(angle - PI * 0.5) - PI * 0.5);
		const float NORMAL_TOLERANCE = PI / 4.0;
		bool sameDirection = myNormal.dot(yourNormal) > 0;
		if (estimator < NORMAL_TOLERANCE && sameDirection) {
			if (matched == NULL || estimator < matchedEstimator) {
				Eigen::Vector4f myCentroid, yourCentroid;
				pcl::compute3DCentroid(*cloud,myCentroid);
				pcl::compute3DCentroid(*polygon->getCloud(),yourCentroid);
				ofVec3f v1 (myCentroid.x(),myCentroid.y(),myCentroid.z());
				ofVec3f v2 (yourCentroid.x(),yourCentroid.y(),yourCentroid.z());

				float distEstimator = abs((v1 - v2).length());
				cout << "size: " << distEstimator << endl;
				if(matched == NULL || (distEstimator < 0.2 && distEstimator < matchedDistance))
				{
					wasRemoved = matched != NULL;
					removed = matched;
					matched = polygon;
					matchedEstimator = estimator;
					matchedDistance = distEstimator;
					
					/*saveCloudAsFile("original" + ofToString(this->getPolygonModelObject()->getName()) + ".pcd", *cloud);
					saveCloudAsFile("matched" + ofToString(this->getPolygonModelObject()->getName()) + "m.pcd", *polygon->getCloud());*/


					////Establezco la transformacion
					//Eigen::Vector4f myCentroid, yourCentroid;
					//pcl::compute3DCentroid(*cloud,myCentroid);
					//pcl::compute3DCentroid(*polygon->getCloud(),yourCentroid);
					//Eigen::Vector4f dif = myCentroid - yourCentroid;
					//matchingTransformation = Eigen::AngleAxisf(angle,Eigen::Vector3f(axis.x,axis.y,axis.z));
					////matchingTransformation = Eigen::Translation3f(Eigen::Vector3f(dif.x(),dif.y(),dif.z()));
					////matchingAngleRotation = angle;
				}
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

	Eigen::Affine3f	PCPolygon::getMatchingTransformation()
	{
		return matchingTransformation;
	}

	void PCPolygon::updateMatching() {
		if (hasMatching()) {
			const float NORMAL_TOLERANCE = PI / 6.0;
			const float DISTANCE_TOLERANCE = 0.005;
			if(matchedEstimator > NORMAL_TOLERANCE || 
			   matchedDistance > DISTANCE_TOLERANCE ||	
			   this->cloud->size() < matched->getCloud()->size())			//Evita perder puntos por oclusión
			{
				//Actualizo Vertices
				vector<PairMatching> bestMatch = bestMatching(
					getPolygonModelObject()->getMathModel().getVertexs(),
					matched->getPolygonModelObject()->getMathModel().getVertexs(),
					distanceBetween);
				vector<ofVec3f> matchedVertexs(matched->getPolygonModelObject()->getMathModel().getVertexs());
				Plane3D matchedPlane = matched->getPolygonModelObject()->getMathModel().getPlane();

				/*for (int i = 0; i < bestMatch.size(); i++) {
					getPolygonModelObject()->setVertex(bestMatch[i].ixA, matchedVertexs.at(bestMatch[i].ixB));
				}*/
				getPolygonModelObject()->setVertexs(matchedVertexs);

				getPolygonModelObject()->getMathModel().setPlane(matchedPlane);

				//actualizo coeficientes
				coefficients.header = matched->coefficients.header;
				coefficients.values = matched->coefficients.values;

				saveCloudAsFile("pcpolygonOri" + ofToString(this->getPolygonModelObject()->getName()) + ".pcd", *cloud);
				saveCloudAsFile("pcpolygonMatch" + ofToString(this->getPolygonModelObject()->getName()) + ".pcd", *matched->getCloud());
			
				//Actualizo nube
				this->cloud = matched->getCloud();
			}
			else
				cout << "mantengo la misma nube - " << ofToString(this->getPolygonModelObject()->getName()) << endl;

			//this->cloud = matched->getCloud();

			saveCloudAsFile("pcpolygonResult" + ofToString(this->getPolygonModelObject()->getName()) + ".pcd", *cloud);


			removeMatching();
		}
	}
	
	void PCPolygon::removeMatching() {
		if (hasMatching()) {
			matched.reset();
			matchedEstimator = MAX_FLOAT;
		}
	}

}
