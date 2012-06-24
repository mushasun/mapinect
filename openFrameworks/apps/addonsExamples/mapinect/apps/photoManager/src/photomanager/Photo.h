#ifndef PHOTO_H__
#define PHOTO_H__

#include "ofVec3f.h"
#include "PCPolygon.h"
#include "PCPolyhedron.h"

using namespace mapinect;

namespace photo {
	class Photo {
		public:
			
			Photo();

			inline void setPos(ofVec3f val){ position = val; }
			inline void setNormal(ofVec3f val){ normal = val; }
			inline void setScale(ofVec3f val){ scale = val; }
			inline void setRotation(float val){ rotation = val; }

			inline ofVec3f getPos (){ return position; }
			inline ofVec3f getNormal (){ return normal;}
			inline ofVec3f getScale (){ return scale;}
			inline float getRotation (){ return rotation;}
			
			void draw();

			float width;
			float height;
		private:
			ofVec3f position;
			ofVec3f normal;
			ofVec3f scale;
			float rotation;

		};
}

#endif	// PHOTO_H__
