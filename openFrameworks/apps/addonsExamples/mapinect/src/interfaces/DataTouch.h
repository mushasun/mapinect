#ifndef DATA_TOUCH_H__
#define DATA_TOUCH_H__

#include "IPolygon.h"

namespace mapinect {

	typedef enum {
		kTouchTypeStarted = 0,
		kTouchTypeHolding,
		kTouchTypeReleased
	} DataTouchType;
	
	struct DataTouch {
		public:
			DataTouch() { }
			DataTouch(const int id, IPolygonPtr polygon, const DataTouchType& type, const ofVec3f& touchPoint)
				: id(id), polygon(polygon), type(type), touchPoint(touchPoint) { }

			const int				getId() const			{ return id; }
			const IPolygonPtr		getPolygon() const		{ return polygon; }
			const DataTouchType&	getType() const			{ return type; }
			const ofVec3f&			getTouchPoint() const	{ return touchPoint; }

		private:
			int				id;
			IPolygonPtr		polygon;
			DataTouchType	type;
			ofVec3f			touchPoint;
	};
}

#endif	// DATA_TOUCH_H__