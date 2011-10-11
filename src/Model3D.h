#ifndef MAPINECT_MODEL3D_H__
#define MAPINECT_MODEL3D_H__

#include "Object3D.h"
#include "ofxMutex.h"

namespace mapinect {
	class Model3D {
	public:
		Model3D();

		ofxMutex			objectsMutex;
		list<Object3D*>		objects;
	};
}

#endif	// PCM_H__
