#include "Plane3D.h"

#include "ofVecUtils.h"

namespace mapinect
{
	Plane3D::Plane3D(const pcl::ModelCoefficients& coefficients)
		: normal(coefficients.values[0], coefficients.values[1], coefficients.values[2]), d(coefficients.values[3])
	{
		d /= normal.length();
		normal.normalize();
	}

	Plane3D::Plane3D(const ofVec3f& point, const ofVec3f& normal)
		: normal(normal)
	{
		this->normal.normalize();
		d = -(normal.dot(point));
	}

	Plane3D::Plane3D(const ofVec3f& p1, const ofVec3f& p2, const ofVec3f& p3)
	{
		normal = computeNormal(p1, p2, p3);
		normal.normalize();
		d = -(normal.dot(p1));
	}

	Plane3D::Plane3D(const Plane3D& plane)
		: normal(plane.normal), d(plane.d)
	{
	}

	pcl::ModelCoefficients Plane3D::getCoefficients() const
	{
		pcl::ModelCoefficients coef;
		coef.values.push_back(normal.x);
		coef.values.push_back(normal.y);
		coef.values.push_back(normal.z);
		coef.values.push_back(d);
		return coef;
	}

	float Plane3D::signedDistance(const ofVec3f& p) const
	{
		// http://paulbourke.net/geometry/pointline/

		return normal.dot(p) + d;
	}

	float Plane3D::distance(const ofVec3f& p) const
	{
		return fabsf(signedDistance(p));
	}

	ofVec3f Plane3D::project(const ofVec3f& p) const
	{
		float t = signedDistance(p);
		ofVec3f result = p - normal * t;
		return result;
	}

	Line3D Plane3D::intersection(const Plane3D& otherPlane) const
	{
		// http://paulbourke.net/geometry/planeplane/
		
		float dot = normal.dot(otherPlane.getNormal());
		if (fabsf(dot) < MATH_EPSILON)
			return Line3D(BAD_OFVEC3F, BAD_OFVEC3F);

		float det = -(dot * dot);
		float c1 = (d - otherPlane.d) / det;
		float c2 = -c1;

		ofVec3f vA(normal * c1 + otherPlane.normal * c2);
		ofVec3f vB(normal * c1 + otherPlane.normal * c2 + normal.getCrossed(otherPlane.normal));

		return Line3D(vA, vB);
	}

	ofVec3f Plane3D::intersection(const Plane3D& otherPlane1, const Plane3D& otherPlane2) const
	{
		// http://paulbourke.net/geometry/3planes/

		float den = normal.dot(otherPlane1.normal.getCrossed(otherPlane2.normal));
		if (fabsf(den) < MATH_EPSILON)
			return BAD_OFVEC3F;

		ofVec3f c1 = d * (otherPlane2.normal.getCrossed(otherPlane1.normal));
		ofVec3f c2 = otherPlane1.d * (normal.getCrossed(otherPlane2.normal));
		ofVec3f c3 = otherPlane2.d * (otherPlane1.normal.getCrossed(normal));

		ofVec3f pto(c1 + c2 + c3);
		pto *= den;

		return pto;

	}

}
