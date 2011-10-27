#include "PCQuadrilateral.h"

#include "Triangle2D.h"
#include "ofxVecUtils.h"

namespace mapinect {

	bool PCQuadrilateral::detectPolygon(const std::vector<ofxVec3f>& vCloud) {
		//ofxVec3f vMin, vMax;
		findOfxVec3fBoundingBox(vCloud, vMin, vMax);
		ofxVec3f center = vMin + vMax;
		center *= 0.5f;
		
		int ixA = 0;
		ofxVec3f vA(vCloud.at(ixA));
		for (int k = 1; k < vCloud.size(); k++) {
			ofxVec3f v(vCloud.at(k));
			if (center.distance(v) > center.distance(vA)) {
				ixA = k;
				vA = v;
			}
		}

		int ixB = 0;
		ofxVec3f vB(vCloud.at(ixB));
		for (int k = 1; k < vCloud.size(); k++) {
			ofxVec3f v(vCloud.at(k));
			if (vA.distance(v) > vA.distance(vB)) {
				ixB = k;
				vB = v;
			}
		}

		DiscardCoordinate discard = calculateDiscardCoordinate(vMin, vMax);

		ofxVec2f v2A = discardCoordinateOfxVec3f(vA, discard);
		ofxVec2f v2B = discardCoordinateOfxVec3f(vB, discard);
		mapinect::Line2D lineAB(v2A, v2B);
		int ixC = 0;
		ofxVec2f v2C(v2A);
		double distanceC = 0;
		for (int k = 0; k < vCloud.size(); k++) {
			ofxVec3f v(vCloud.at(k));
			ofxVec2f v2 = discardCoordinateOfxVec3f(v, discard);
			double distance = lineAB.distance(v2);
			if (distance > distanceC) {
				distanceC = distance;
				ixC = k;
				v2C = v2;
			}
		}
		
		//cout << "max distance to line: " << distanceC << endl;

		int ixD = ixC;
		ofxVec2f v2D(v2C);
		mapinect::Triangle2D triangleABC(v2A, v2B, v2C);
		double distanceD = 0;
		for (int k = 0; k < vCloud.size(); k++) {
			ofxVec3f v(vCloud.at(k));
			ofxVec2f v2 = discardCoordinateOfxVec3f(v, discard);
			double distance = triangleABC.distance(v2);
			if (distance > distanceD) {
				distanceD = distance;
				ixD = k;
				v2D = v2;
			}
		}

		//cout << "max distance to triangle: " << distanceD << endl;

		getPolygonModelObject()->addVertex(vCloud.at(ixA));
		getPolygonModelObject()->addVertex(vCloud.at(ixB));
		getPolygonModelObject()->addVertex(vCloud.at(ixC));
		getPolygonModelObject()->addVertex(vCloud.at(ixD));
		getPolygonModelObject()->sortVertexs();

		std::vector<int> indices (4);
		indices[0] = ixA;
		indices[1] = ixB;
		indices[2] = ixC;
		indices[3] = ixD;
		pcl::PointIndices::Ptr v (new pcl::PointIndices ());
		vertexIdxs = v;
		vertexIdxs->indices = indices;


		//cout << "x: " << pVA.x << " y: " << pVA.y << " z: " << pVA.z << endl;
		return true;
	}

	bool PCQuadrilateral::detectPolygon2(const std::vector<ofxVec3f>& vCloud) {
		ofxVec3f vMinMin = ofxVec3f(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
		ofxVec3f vMaxMax = ofxVec3f(-MAX_FLOAT, -MAX_FLOAT, -MAX_FLOAT);
		
		ofxVec3f vMinMax = ofxVec3f(MAX_FLOAT, -MAX_FLOAT, MAX_FLOAT);
		ofxVec3f vMaxMin = ofxVec3f(-MAX_FLOAT, MAX_FLOAT, -MAX_FLOAT);

		int pMinMin = -1;
		int pMaxMax = -1;
		int pMaxMin = -1;
		int pMinMax = -1;

		for (int k = 0; k < vCloud.size(); k++) {
			ofxVec3f p = vCloud.at(k);

			//Encuentro los vertices con:
			//menor 'x' y menor 'y'  --> vMinMin 
			//mayor 'x' y mayor 'y' --> vMaxMax
			//mayor 'x' y menor 'y' --> vMaxMin
			//menor 'x' y mayor 'y' --> vMinMax

			if(p.x > vMaxMax.x && p.y > vMaxMax.y)
			{
				vMaxMax.x = p.x;
				vMaxMax.y = p.y;
				pMaxMax = k;
			}

			if(p.x > vMaxMin.x && p.y < vMaxMin.y)
			{
				vMaxMin.x = p.x;
				vMaxMin.y = p.y;
				pMaxMin = k;
			}

			if(p.x < vMinMin.x && p.y < vMinMin.y)
			{
				vMinMin.x = p.x;
				vMinMin.y = p.y;
				pMinMin = k;

				//cout << "TempMinMin: " << p.x << ", " << p.y << endl;
			}

			if(p.x < vMinMax.x && p.y > vMinMax.y)
			{
				vMinMax.x = p.x;
				vMinMax.y = p.y;
				pMinMax = k;
			}
		}
	
		if (pMaxMax != pMinMin != pMaxMin != pMinMax != -1)
		{
			getPolygonModelObject()->addVertex(vCloud.at(pMinMin));
			getPolygonModelObject()->addVertex(vCloud.at(pMaxMin));
			getPolygonModelObject()->addVertex(vCloud.at(pMaxMax));
			getPolygonModelObject()->addVertex(vCloud.at(pMinMax));

			//cout << "MinMin: " << pVA.x << ", " << pVA.y << endl;
			//cout << "MaxMin: " << pVB.x << ", " << pVB.y << endl;
			//cout << "MaxMax: " << pVC.x << ", " << pVC.y << endl;
			//cout << "MinMax: " << pVD.x << ", " << pVD.y << endl;
			return true;
		}
		else 
			return false;
	}
}
