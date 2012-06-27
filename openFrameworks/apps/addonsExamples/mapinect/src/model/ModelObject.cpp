#include "ModelObject.h"

#include "ofGraphicsUtils.h"

namespace mapinect {
	ModelObject::ModelObject() : vCenter(0, 0, 0), vScale(1, 1, 1), vRotation(0, 0, 0), color(kRGBWhite) {

	}

	void ModelObject::drawObject() {
		ofPushMatrix();
			ofSetColor(color);
			draw();
		ofPopMatrix();
	}
}
