#ifndef MAPINECT_QUAD3D_H__
#define MAPINECT_QUAD3D_H__

#include "ofxVec3f.h"
#include <vector>

namespace mapinect {

	class Quad3D {
	public:
		Quad3D();
		Quad3D(const ofxVec3f &vA, const ofxVec3f &vB, const ofxVec3f &vC, const ofxVec3f &vD);
		virtual ~Quad3D() { };

		bool findQuad(const std::vector<ofxVec3f>& vCloud);

		inline const ofxVec3f& getVA() { return pVA; }
		inline const ofxVec3f& getVB() { return pVB; }
		inline const ofxVec3f& getVC() { return pVC; }
		inline const ofxVec3f& getVD() { return pVD; }

	private:
		ofxVec3f pVA;
		ofxVec3f pVB;
		ofxVec3f pVC;
		ofxVec3f pVD;

	};
}

#endif	// MAPINECT_QUAD3D_H__