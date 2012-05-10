#ifndef MAPINECT_Model_H__
#define MAPINECT_Model_H__

#include "ModelObject.h"
#include "ofxMutex.h"
#include <vector>
#include "Table.h"

namespace mapinect {
	class Model {

	public:
		Model();

		ModelObjectPtr					getObjectAt(int index);

		ofxMutex						objectsMutex;
		vector<ModelObjectPtr>			objects;
		TablePtr						table;
	};
}

#endif	// PCM_H__
