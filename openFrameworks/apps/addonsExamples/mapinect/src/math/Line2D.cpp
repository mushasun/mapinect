#include "Line2D.h"

#include "ofVecUtils.h"
#include "utils.h"

namespace mapinect {

	Line2D::Line2D(const ofVec2f &origin, const ofVec2f &destination)
		: origin(origin), destination(destination)
	{
		//assert(origin.distance(destination) > MATH_EPSILON);
		direction = destination - origin;
		a = - direction.y;
		b = direction.x;
		c = origin.x * direction.y - origin.y * direction.x;
		sqrtA2B2 = sqrt(a * a + b * b);
	}

	float Line2D::distance(const ofVec2f& p) const
	{
		ofVec2f projected(projectTo(p));
		return p.distance(projected);
	}

	float Line2D::projectedK(const ofVec2f& p) const
	{
		float num = direction.dot(p - origin);
		float den = direction.dot(direction);
		return num / den;
	}

	ofVec2f Line2D::calculateValue(float k) const
	{
		return origin + direction * k;
	}

	ofVec2f Line2D::projectTo(const ofVec2f& p) const
	{
		float k = projectedK(p);
		return calculateValue(k);
	}

	bool Line2D::isInLine(const ofVec2f& p) const
	{
		return p.distance(projectTo(p)) < MATH_EPSILON;
	}

	bool Line2D::isInSegment(const ofVec2f& p) const
	{
		float k = projectedK(p);
		return p.distance(calculateValue(k)) < MATH_EPSILON && isInSegment(k);
	}

	bool Line2D::isInSegment(float k) const
	{
		return inRange(k, 0.0f, 1.0f);
	}

	float Line2D::segmentLength() const
	{
		return (destination - origin).length();
	}

	float Line2D::evaluate(const ofVec2f& p) const
	{
		return a * p.x + b * p.y + c;
	}

	PositionToLine Line2D::positionTo(const ofVec2f& p) const
	{
		float value = evaluate(p);
		if (abs(value) < MATH_EPSILON)
		{
			return kPositionedInLine;
		}
		else if (value < 0)
		{
			return kPositionedAtLeft;
		}
		else
		{
			return kPositionedAtRight;
		}		
	}

	Line2D Line2D::parallelLineThrough(const ofVec2f& p) const
	{
		ofVec2f pDestination(p + (destination - origin));
		return Line2D(p, pDestination);
	}

	ofVec2f Line2D::intersection(const Line2D& l) const
	{
		ofVec2f dif(destination - origin);
		ofVec2f ldif(l.destination - l.origin);
		float den = ldif.y * dif.x - ldif.x * dif.y;
		if (abs(den) < MATH_EPSILON)
			return BAD_OFVEC2F;
		float numA = ldif.x * (l.origin.y - origin.y) - ldif.y * (origin.x - l.origin.x);
		
		ofVec2f result(origin + (dif * numA / den));

		return result;
	}

}
