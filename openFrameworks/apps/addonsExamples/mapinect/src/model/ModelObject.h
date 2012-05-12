#ifndef MAPINECT_MODELOBJECT_H__
#define MAPINECT_MODELOBJECT_H__

#include "IObject.h"

#include "mapinectTypes.h"

namespace mapinect {
	
	class ModelObject;

	typedef boost::shared_ptr<ModelObject> ModelObjectPtr;

	class ModelObject : public IObject {
		public:
			ModelObject();
			virtual ~ModelObject(void) { }

			void drawObject();
			virtual void draw() = 0;

			inline const ofVec3f&	getCenter()								{ return vCenter; }
			inline void				setCenter(const ofVec3f& center)		{ vCenter = center; }
			inline const ofVec3f&	getScale()								{ return vScale; }
			inline void				setScale(const ofVec3f& scale)			{ vScale = scale; }
			inline const ofVec3f&	getRotation()							{ return vRotation; }
			inline void				setRotation(const ofVec3f& rotation)	{ vRotation = rotation; }
			inline int				getColor()								{ return color; }
			inline void				setColor(int color)						{ this->color = color; }
			inline int				getId()									{ return objId; }
			inline void				setId(int id)							{ objId = id; }

		private:
			ofVec3f					vCenter, vScale, vRotation;
			int						color;
			int						objId;
		
	};
}

#endif	// MAPINECT_MODELOBJECT_H__