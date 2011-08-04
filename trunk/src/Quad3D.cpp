#include "Quad3D.h"
#include "Triangle2D.h"
#include "ofxVecUtils.h"

namespace mapinect {

	Quad3D::Quad3D() { }

	Quad3D::Quad3D(const ofxVec3f &vA, const ofxVec3f &vB, const ofxVec3f &vC, const ofxVec3f &vD)
		: pVA(vA), pVB(vB), pVC(vC), pVD(vD) { }

	bool Quad3D::findQuad(const std::vector<ofxVec3f>& vCloud) {
		ofxVec3f vMin, vMax;
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
		for (int k = 0; k < vCloud.size(); k++) {
			ofxVec3f v(vCloud.at(k));
			ofxVec2f v2 = discardCoordinateOfxVec3f(v, discard);
			if (lineAB.distance(v2) > lineAB.distance(v2C) && lineAB.calculateValue(v2) > 0) {
				ixC = k;
				v2C = v2;
			}
		}
		
		int ixD = ixC;
		ofxVec2f v2D(v2C);
		mapinect::Triangle2D triangleABC(v2A, v2B, v2C);
		float distanceD = 0;
		for (int k = 0; k < vCloud.size(); k++) {
			ofxVec3f v(vCloud.at(k));
			ofxVec2f v2 = discardCoordinateOfxVec3f(v, discard);
			float distance = triangleABC.distance(v2);
			if (distance > distanceD) {
				distanceD = distance;
				ixD = k;
				v2D = v2;
			}
		}

		pVA = vCloud.at(ixA);
		pVB = vCloud.at(ixB);
		pVC = vCloud.at(ixC);
		pVD = vCloud.at(ixD);

		return true;
	}


}
