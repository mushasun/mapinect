#ifndef MAPINECT_QUAD3D_H__
#define MAPINECT_QUAD3D_H__

#include "ofxVec3f.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <vector>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include "pointUtils.h"

namespace mapinect {

	class Quad3D {
	public:
		Quad3D();
		Quad3D(const ofxVec3f &vA, const ofxVec3f &vB, const ofxVec3f &vC, const ofxVec3f &vD);
		virtual ~Quad3D() { };

		bool findQuad(const std::vector<ofxVec3f>& vCloud);
		bool findQuad2(const std::vector<ofxVec3f>& vCloud);

		/*inline const ofxVec3f& getVA() { return POINTXYZ_OFXVEC3F(cloudQuad.at(vertexIdxs->indices.at(0))); }
		inline const ofxVec3f& getVB() { return POINTXYZ_OFXVEC3F(cloudQuad.at(vertexIdxs->indices.at(1))); }
		inline const ofxVec3f& getVC() { return POINTXYZ_OFXVEC3F(cloudQuad.at(vertexIdxs->indices.at(2))); }
		inline const ofxVec3f& getVD() { return POINTXYZ_OFXVEC3F(cloudQuad.at(vertexIdxs->indices.at(3))); }*/
		inline const ofxVec3f& getVA() { return pVA; }
		inline const ofxVec3f& getVB() { return pVB; }
		inline const ofxVec3f& getVC() { return pVC; }
		inline const ofxVec3f& getVD() { return pVD; }

		inline const pcl::PointIndices::Ptr					getVertexIdxs() { return vertexIdxs; }


		ofxVec3f							avgNormal;
		ofxVec3f							vertexNormal;
		pcl::PointCloud<pcl::PointXYZ>::Ptr	cloudQuad;
		pcl::PointCloud<pcl::PointXYZ>::Ptr extendedcloudQuad;
		ofxVec3f							vMin, vMax;
		
	private:
		ofxVec3f pVA;
		ofxVec3f pVB;
		ofxVec3f pVC;
		ofxVec3f pVD;

		
		pcl::PointIndices::Ptr				vertexIdxs;
	};
}

#endif	// MAPINECT_QUAD3D_H__