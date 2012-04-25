#ifndef MAPINECT_Model_H__
#define MAPINECT_Model_H__

#include "ModelObject.h"
#include "ofxMutex.h"
#include <list>

namespace mapinect {
	class Model {
	public:
		Model();

		ModelObject*				getObjectAt(int index);

		ofxMutex					objectsMutex;
		std::list<ModelObject*>		objects;
		ModelObject*				table;
	};
}

#endif	// PCM_H__
