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

	PCPolygon::PCPolygon(PCModelObject* container, const pcl::ModelCoefficients& coefficients,
		const PCPtr& cloud, int objId, bool estimated)
			: PCModelObject(cloud, objId)
	{
		// Point the normal towards the viewpoint
		this->container = container;
		this->coefficients = coefficients;
		this->estimated = estimated;
		if(!estimated)
		{
			ofVec3f coefficientsNormal(::getNormal(this->coefficients));
			pcl::flipNormalTowardsViewpoint(OFVEC3F_PCXYZ(getCenter()),
										container->getCenter().x, container->getCenter().y, container->getCenter().z,
										coefficientsNormal.x, coefficientsNormal.y, coefficientsNormal.z);
			coefficientsNormal *= -1;
			
			if(coefficientsNormal != ::getNormal(coefficients))
			{
				Plane3D plane(getCenter(), coefficientsNormal);
				this->coefficients = plane.getCoefficients();
			}
		}
		matchedDistance = numeric_limits<float>::max();
		matchedEstimator= numeric_limits<float>::max();
		modelObject = ModelObjectPtr(new Polygon(this->coefficients));

		modelObject->setId(getId());
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
		ofVec3f normal(::getNormal(coefficients));
		normal.normalize();
		return normal;
	}

	void PCPolygon::draw() {
		PCModelObject::draw();
		if (drawPointCloud) {
			if (getPolygonModelObject() != NULL) {
				ofSetColor(kRGBRed);
				ofVec3f avg = computeCentroid(getPolygonModelObject()->getMathModel().getVertexs());
				ofVec3f sAvg = getScreenCoords(avg);
				ofDrawBitmapString(ofToString(getPolygonModelObject()->getName()), sAvg.x, sAvg.y, 0);
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

	bool PCPolygon::matches(const PCPolygonPtr& polygon, PCPolygonPtr& removed, bool& wasRemoved)
	{
		bool result = false;
		wasRemoved = false;
		
		ofVec3f myNormal = getNormal();
		ofVec3f yourNormal = polygon->getNormal();
		ofVec3f axis;
		{
			gModel->tableMutex.lock();
			axis = gModel->getTable()->getNormal();
			gModel->tableMutex.unlock();

		}
		float angle = acos(myNormal.dot(yourNormal));
		float estimator = fabsf(fabsf(angle - PI * 0.5) - PI * 0.5);
		const float NORMAL_TOLERANCE = PI / 4.0;
		bool sameDirection = myNormal.dot(yourNormal) > 0;
		if (estimator < NORMAL_TOLERANCE && sameDirection) {
			if (estimator < matchedEstimator) {
				Eigen::Vector4f myCentroid, yourCentroid;
				pcl::compute3DCentroid(*cloud,myCentroid);
				pcl::compute3DCentroid(*polygon->getCloud(),yourCentroid);
				ofVec3f v1 (myCentroid.x(),myCentroid.y(),myCentroid.z());
				ofVec3f v2 (yourCentroid.x(),yourCentroid.y(),yourCentroid.z());

				float distEstimator = abs((v1 - v2).length());
				//cout << "size: " << distEstimator << endl;
				if((distEstimator < 0.05 && distEstimator < matchedDistance))
				{
					wasRemoved = matched != NULL;
					removed = matched;
					matched = polygon;
					matchedEstimator = estimator;
					matchedDistance = distEstimator;
					
					saveCloud("original" + ofToString(this->getPolygonModelObject()->getName()) + ".pcd", *cloud);
					saveCloud("matched" + ofToString(this->getPolygonModelObject()->getName()) + "m.pcd", *polygon->getCloud());

					result = true;
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

				saveCloud("pcpolygonOri" + ofToString(this->getPolygonModelObject()->getName()) + ".pcd", *cloud);
				saveCloud("pcpolygonMatch" + ofToString(this->getPolygonModelObject()->getName()) + ".pcd", *matched->getCloud());
			
				//Actualizo nube
				this->cloud = matched->getCloud();
			}
			/*else
				cout << "mantengo la misma nube - " << ofToString(this->getPolygonModelObject()->getName()) << endl;
*/
			//this->cloud = matched->getCloud();

			saveCloud("pcpolygonResult" + ofToString(this->getPolygonModelObject()->getName()) + ".pcd", *cloud);


			removeMatching();
		}
	}
	
	void PCPolygon::removeMatching() {
		if (hasMatching()) {
			matched.reset();
			matchedEstimator = MAX_FLOAT;
			matchedDistance = MAX_FLOAT;
		}
	}

}
