#include "TrackedTouch.h"

namespace mapinect
{

	static int gId = 0;

	TrackedTouch::TrackedTouch(const IPolygonPtr& polygon, const ofVec3f& point)
		: id(gId++), status(kTouchTypeStarted), polygon(polygon), point(point)
	{
		removeMatching();
	}

	DataTouch TrackedTouch::getDataTouch() const
	{
		return DataTouch(id, polygon, status, point);
	}

	void TrackedTouch::removeMatching()
	{
		 matchingTouch.reset();
		 nearest = MAX_FLOAT;
	}
	
	bool TrackedTouch::matches(const TrackedTouchPtr& tracked, TrackedTouchPtr& removed, bool &wasRemoved)
	{
		wasRemoved = false;
		if (polygon->getId() == tracked->polygon->getId())
		{
			ofVec3f translation(tracked->point - point);
			if(translation.length() < nearest)
			{
				nearest = translation.length();
				if(matchingTouch.get() != NULL)
				{
					removed = matchingTouch;
					wasRemoved = true;
				}
				matchingTouch = tracked;
				return true;
			}
		}
		return false;
	}

	void TrackedTouch::updateMatching()
	{
		if (hasMatching())
		{
			status = kTouchTypeHolding;
			point = matchingTouch->point;
			polygon = matchingTouch->polygon;
			removeMatching();
		}
		else if (status == kTouchTypeHolding)
		{
			status = kTouchTypeReleased;
		}
	}

}