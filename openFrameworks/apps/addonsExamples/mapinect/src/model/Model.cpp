#include "Model.h"

namespace mapinect {
	Model::Model() {
		this->table = NULL;
	}

	ModelObject* Model::getObjectAt(int index) {
		list<ModelObject*>::iterator iter = objects.begin();
		while (index > 0 && iter != objects.end()) {
			iter++;
			index--;
		}
		if (index > 0) {
			return NULL;
		}
		else {
			return *iter;
		}
	}

}
