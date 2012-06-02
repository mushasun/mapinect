#include "Line2D.h"
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
		return 0 <= k && k <= 1;
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

}
