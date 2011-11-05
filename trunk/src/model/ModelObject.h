#ifndef MAPINECT_ModelObject_H__
#define MAPINECT_ModelObject_H__

#include "ofxVec3f.h"
#include "ofGraphicsUtils.h"

namespace mapinect {
	class ModelObject {
		public:
			ModelObject();
			virtual ~ModelObject(void) { }

			void drawObject();
			virtual void draw() = 0;

			inline const ofxVec3f	getCenter()								{ return vCenter; }
			inline void				setCenter(const ofxVec3f& center)		{ vCenter = center; }
			inline const ofxVec3f	getScale()								{ return vScale; }
			inline void				setScale(const ofxVec3f& scale)			{ vScale = scale; }
			inline const ofxVec3f	getRotation()							{ return vRotation; }
			inline void				setRotation(const ofxVec3f& rotation)	{ vRotation = rotation; }
			inline int				getId()									{ return objId; }
			inline void				setId(int id)							{ objId = id; }

		private:
			ofxVec3f				vCenter, vScale, vRotation;
			int						color;
			int						objId;
		
	};
}

#endif	// MAPINECT_ModelObject_H__