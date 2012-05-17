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
			DataTouch(const int id, IPolygon* polygon, const DataTouchType& type, const ofVec3f& touchPoint)
				: id(id), polygon(polygon), type(type), touchPoint(touchPoint) { }

			const int				getId()			{ return id; }
			const IPolygon*			getPolygon()	{ return polygon; }
			const DataTouchType&	getType()		{ return type; }
			const ofVec3f&			getTouchPoint()	{ return touchPoint; }

		private:
			int				id;
			IPolygon*		polygon;
			DataTouchType	type;
			ofVec3f			touchPoint;
	};
}

#endif	// DATA_TOUCH_H__