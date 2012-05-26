#include "Line3D.h"
#include "utils.h"

namespace mapinect {

	Line3D::Line3D(const ofVec3f &origin, const ofVec3f &destination)
		: origin(origin), destination(destination)
	{
		valid = origin.distance(destination) > MATH_EPSILON;
		direction = destination - origin;
	}

	float Line3D::distance(const ofVec3f& p) const
	{
		ofVec3f projected(projectTo(p));
		return p.distance(projected);
	}

	float Line3D::projectedK(const ofVec3f& p) const
	{
		float num = direction.dot(p - origin);
		float den = direction.dot(direction);
		return num / den;
	}

	ofVec3f Line3D::calculateValue(float k) const
	{
		return origin + direction * k;
	}

	ofVec3f Line3D::projectTo(const ofVec3f& p) const
	{
		float k = projectedK(p);
		return calculateValue(k);
	}

	bool Line3D::isInLine(const ofVec3f& p) const
	{
		return p.distance(projectTo(p)) < MATH_EPSILON;
	}

	bool Line3D::isInSegment(const ofVec3f& p) const
	{
		float k = projectedK(p);
		return p.distance(calculateValue(k)) < MATH_EPSILON && isInSegment(k);
	}

	bool Line3D::isInSegment(float k) const
	{
		return 0 <= k && k <= 1;
	}

}
