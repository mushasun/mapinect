#ifndef MAPINECT_PLANE3D_H__
#define MAPINECT_PLANE3D_H__

#include "ofVec3f.h"
#include <pcl/ModelCoefficients.h>
#include "Line3D.h"

namespace mapinect
{
	class Plane3D
	{
	public:
		Plane3D() { }
		Plane3D(const pcl::ModelCoefficients& coefficients);
		Plane3D(const ofVec3f& point, const ofVec3f& normal);
		Plane3D(const ofVec3f& p1, const ofVec3f& p2, const ofVec3f& p3);
		Plane3D(const Plane3D& plane);
		virtual ~Plane3D() { };

		float					signedDistance(const ofVec3f& v) const;
		float					distance(const ofVec3f& v) const;
		ofVec3f					project(const ofVec3f& p) const;
		Line3D					intersection(const Plane3D& otherPlane) const;
		ofVec3f					intersection(const Plane3D& otherPlane1, const Plane3D& otherPlane2) const;
		ofVec3f					intersection(const Line3D& line) const;


		inline const ofVec3f&	getNormal() const						{ return normal; }
		pcl::ModelCoefficients	getCoefficients() const;

		bool					isPerpendicular(const Plane3D& plane) const;

	private:
		ofVec3f					normal;
		float					d;

	};
}

#endif	// MAPINECT_PLANE3D_H__
