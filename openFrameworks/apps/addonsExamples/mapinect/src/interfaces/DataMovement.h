#ifndef DATA_MOVEMENT_H__
#define DATA_MOVEMENT_H__

#include "ofVec3f.h"

namespace mapinect {

	struct DataMovement {
		public:
			DataMovement(const ofVec3f& translation, const ofVec3f& rotation)
				: translation(translation), rotation(rotation) { }

			const ofVec3f&		getTranslation() const	{ return translation; }
			const ofVec3f&		getRotation() const		{ return rotation; }

		private:
			ofVec3f				translation;
			ofVec3f				rotation;
	};
}

#endif	// DATA_MOVEMENT_H__