#include "ModelObject.h"

namespace mapinect {
	ModelObject::ModelObject() : vCenter(0, 0, 0), vScale(1, 1, 1), vRotation(0, 0, 0), color(kRGBWhite) {

	}

	void ModelObject::drawObject() {
		ofPushMatrix();
			ofSetColor(color);
			ofTranslate(vCenter.x, vCenter.y, vCenter.z);
			ofScale(vScale.x, vScale.y, vScale.z);
			ofRotateX(vRotation.x);
			ofRotateY(vRotation.y);
			ofRotateZ(vRotation.z);

			draw();
		ofPopMatrix();
	}
}
