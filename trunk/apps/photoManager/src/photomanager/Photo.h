#ifndef PHOTO_H__
#define PHOTO_H__

#include "ofxVec3f.h"
#include "PCPolygon.h"
#include "PCPolyhedron.h"
#include "ITxManager.h"

using namespace mapinect;

namespace photo {
	class Photo {
		public:
			
			Photo();

			inline void setPos(ofxVec3f val){ position = val; }
			inline void setNormal(ofxVec3f val){ normal = val; }
			inline void setScale(ofxVec3f val){ scale = val; }
			inline void setRotation(float val){ rotation = val; }

			inline ofxVec3f getPos (){ return position; }
			inline ofxVec3f getNormal (){ return normal;}
			inline ofxVec3f getScale (){ return scale;}
			inline float getRotation (){ return rotation;}
			
			void draw(const ITxManager* txManager);

			GLuint	texture;
			float width;
			float height;
		private:
			ofxVec3f position;
			ofxVec3f normal;
			ofxVec3f scale;
			float rotation;

		};
}

#endif	// PHOTO_H__
