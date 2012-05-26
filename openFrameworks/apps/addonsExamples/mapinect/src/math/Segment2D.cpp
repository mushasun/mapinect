#include "Segment2D.h"

#include "utils.h"

namespace mapinect {

	Segment2D::Segment2D(const ofVec2f& origin, const ofVec2f& destination)
		: Line2D(origin, destination), pDestination(destination)
	{
	}

	bool Segment2D::isInSegment(const ofVec2f& p) const
	{
		if (distance(p) > MATH_EPSILON)
		{
			return false;
		}
		if ((inRange(p.x, getOrigin().x, pDestination.x) || inRange(p.x, pDestination.x, getOrigin().x))
			&& (inRange(p.y, getOrigin().y, pDestination.y) || inRange(p.y, pDestination.y, getOrigin().y)))
		{
			return true;
		}
		return false;
	}

	float Segment2D::directionScale(const ofVec2f& p) const
	{
		assert(isInSegment(p));
		if (fabsf(pDestination.x - getOrigin().x) > MATH_EPSILON)
		{
			return (p.x - getOrigin().x) / (pDestination.x - getOrigin().x);
		}
		else
		{
			return (p.y - getOrigin().y) / (pDestination.y - getOrigin().y);
		}
	}

}
