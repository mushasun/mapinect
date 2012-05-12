#ifndef I_OBJECT_H__
#define I_OBJECT_H__

#include "ofVec3f.h"

namespace mapinect {
	
	class IObject {

		public:
			virtual const ofVec3f&	getCenter() = 0;
			virtual const ofVec3f&	getScale() = 0;
			virtual const ofVec3f&	getRotation() = 0;
			virtual int				getColor() = 0;
			virtual int				getId() = 0;

	};
}

#endif	// I_OBJECT_H__