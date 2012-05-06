#ifndef MAPINECT_Model_H__
#define MAPINECT_Model_H__

#include "ModelObject.h"
#include "ofxMutex.h"
#include <list>

namespace mapinect {
	class Table;

	class Model {

	public:
		Model();

		ModelObject*				getObjectAt(int index);

		ofxMutex					objectsMutex;
		std::list<ModelObject*>		objects;
		Table*						table;
	};
}

#endif	// PCM_H__
