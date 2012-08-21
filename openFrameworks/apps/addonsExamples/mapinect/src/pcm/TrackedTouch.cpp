#include "TrackedTouch.h"
#include "Constants.h"

namespace mapinect
{
	static int gId = 0;

	TrackedTouch::TrackedTouch(const IPolygonPtr& polygon, const ofVec3f& point)
		: id(gId++), status(kTouchTypeStarted), polygon(polygon), point(point), lifeCounter(2)
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
	
	bool TrackedTouch::confirmMatch(const TrackedTouchPtr& tracked, TrackedTouchPtr& removed)
	{
		bool result = false;

		if(matchingTouch != NULL)
		{
			removed = matchingTouch;
			result = true;
		}
		nearest = (tracked->point - point).length();
		matchingTouch = tracked;

		return result;
	}

	float TrackedTouch::matchingTrackedTouch(const TrackedTouchPtr& tracked) const
	{
		float result = numeric_limits<float>::max();
		if (polygon->getId() == tracked->polygon->getId())
		{
			float translationLen = (tracked->point - point).length();
			if(translationLen < nearest && translationLen < Constants::TOUCH_TRANSLATION_TOLERANCE())
			{
				result = translationLen;
			}
		}
		return result;
	}

	bool TrackedTouch::updateMatching()
	{
		bool reportStatus = false;

		if (hasMatching())
			lifeCounter++;
		else
			lifeCounter--;

		bool changing = false;
		if (status == kTouchTypeStarted)
		{
			if (lifeCounter == Constants::OBJECT_FRAMES_TO_ACCEPT + 1)
				changing = true;
			else if (lifeCounter == 0)
				status = kTouchTypeReleased;
		}

		if (status == kTouchTypeHolding || changing)
		{
			reportStatus = true;
			if (hasMatching())
			{
				lifeCounter = Constants::TOUCH_FRAMES_TO_DISCARD;
				if (point == matchingTouch->point)
				{
					reportStatus = false;
				}
				else
				{
					point = matchingTouch->point;
				}
				polygon = matchingTouch->polygon;
				removeMatching();
			}
			else
			{
				if (lifeCounter == 0)
				{
					status = kTouchTypeReleased;
				}
			}
		}
		return reportStatus;
	}

	void TrackedTouch::updateToHolding()
	{
		if (status == kTouchTypeStarted && lifeCounter >= Constants::OBJECT_FRAMES_TO_ACCEPT + 1)
			status = kTouchTypeHolding;
	}

}