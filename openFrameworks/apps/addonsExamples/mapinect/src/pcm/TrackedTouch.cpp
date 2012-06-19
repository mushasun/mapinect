#include "TrackedTouch.h"

namespace mapinect
{

#define kTrackedTouchLifeFrames		3

	static int gId = 0;

	TrackedTouch::TrackedTouch(const IPolygonPtr& polygon, const ofVec3f& point)
		: id(gId++), status(kTouchTypeStarted), polygon(polygon), point(point), lifeCounter(kTrackedTouchLifeFrames)
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

	bool TrackedTouch::updateMatching()
	{
		bool hasChanged = true;
		if (hasMatching())
		{
			status = kTouchTypeHolding;
			lifeCounter = kTrackedTouchLifeFrames;
			if (point == matchingTouch->point)
			{
				hasChanged = false;
			}
			else
			{
				point = matchingTouch->point;
			}
			polygon = matchingTouch->polygon;
			removeMatching();
		}
		else if (status == kTouchTypeHolding)
		{
			lifeCounter--;
			if (lifeCounter == 0)
			{
				status = kTouchTypeReleased;
			}
		}
		return hasChanged;
	}

	void TrackedTouch::updateToHolding()
	{
		status = kTouchTypeHolding;
	}

}