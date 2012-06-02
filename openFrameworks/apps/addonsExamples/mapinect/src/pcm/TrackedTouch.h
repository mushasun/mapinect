#ifndef MAPINECT_TRACKEDTOUCH_H__
#define MAPINECT_TRACKEDTOUCH_H__

#include "DataTouch.h"
#include "Polygon3D.h"

namespace mapinect
{

	class TrackedTouch;

	typedef boost::shared_ptr<TrackedTouch> TrackedTouchPtr;

	class TrackedTouch {
	
	public:
		TrackedTouch(const IPolygonPtr& polygon, const ofVec3f& point);
		virtual ~TrackedTouch() { };

		inline	int					getId() const						{ return id; }
		inline	DataTouchType		getTouchStatus() const				{ return status; }
		inline	const IPolygonPtr&	getPolygon() const					{ return polygon; }
		inline	const ofVec3f&		getTrackedPoint() const				{ return point; }
		inline	bool				hasMatching() const					{ return matchingTouch != NULL; }

				DataTouch			getDataTouch() const;
		inline	IObjectPtr			getObject() const					{ return polygon->getContainer(); }

				bool				matches(const TrackedTouchPtr& tracked, TrackedTouchPtr& removed, bool &wasRemoved);
				void				updateMatching();

				void				removeMatching();

	private:
		int							id;
		DataTouchType				status;
		IPolygonPtr					polygon;
		ofVec3f						point;
		TrackedTouchPtr				matchingTouch;

		float						nearest;
	};

}
#endif	// MAPINECT_TRACKEDTOUCH_H__
