#ifndef MAPINECT_MODELOBJECT_H__
#define MAPINECT_MODELOBJECT_H__

#include "mapinectTypes.h"
#include "ofColor.h"

namespace mapinect {
	
	class ModelObject;

	typedef boost::shared_ptr<ModelObject> ModelObjectPtr;

	class ModelObject {
		public:
			ModelObject();
			virtual ~ModelObject(void) { }

			void drawObject();
			virtual void draw() = 0;

			inline virtual const ofVec3f&	getCenter() const						{ return vCenter; }
			inline void						setCenter(const ofVec3f& center)		{ vCenter = center; }
			inline virtual const ofVec3f&	getScale() const						{ return vScale; }
			inline void						setScale(const ofVec3f& scale)			{ vScale = scale; }
			inline virtual const ofVec3f&	getRotation() const						{ return vRotation; }
			inline void						setRotation(const ofVec3f& rotation)	{ vRotation = rotation; }
			inline ofColor					getColor() const 						{ return color; }
			inline void						setColor(const ofColor& color)			{ this->color = color; }
			inline virtual int				getId() const							{ return objId; }
			inline void						setId(int id)							{ objId = id; }

		private:
			ofVec3f							vCenter, vScale, vRotation;
			ofColor							color;
			int								objId;
		
	};
}

#endif	// MAPINECT_MODELOBJECT_H__