#include "Model.h"

namespace mapinect {
	Model::Model() { }

	ModelObjectPtr Model::getObjectAt(int index) {
		vector<ModelObjectPtr>::iterator iter = objects.begin();
		while (index > 0 && iter != objects.end()) {
			iter++;
			index--;
		}
		if (index > 0) {
			return ModelObjectPtr();
		}
		else {
			return *iter;
		}
	}

}
