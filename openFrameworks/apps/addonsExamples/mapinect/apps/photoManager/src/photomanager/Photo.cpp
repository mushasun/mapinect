#include "Photo.h"

#include "ofGraphicsUtils.h"

using namespace mapinect;
namespace photo {
	Photo::Photo()
	{
		this->position = ofVec3f(0,0,0);
		this->normal = ofVec3f(0,1,0);
		this->scale = ofVec3f(1,1,1);
		this->rotation = 0;
	}
	void Photo::draw()
	{
		ofPushMatrix();

		////Hallo la rotación http://www.gamedev.net/topic/472246-rotation-matrix-between-two-vectors/
		ofVec3f w (normal);
		w.cross(ofVec3f(0,0,1));

		float angle = (asin(w.length())* 180) / PI;
		w.normalize();
		

		ofTranslate(position.x,position.y,position.z);
		ofRotate(rotation,normal.x,normal.y,normal.z);
		ofScale(scale.x,scale.y,scale.z);
		ofRotate(-angle,1,0,0);

		// Bind Texture

		
		ofVec3f v1, v2,v3,v4;
		v1 = ofVec3f();
		v2 = v1;
		v2.x += width;
		v3 = v2;
		v3.y += height;
		v4 = ofVec3f();
		v4.y += height;

		ofDrawQuadTextured(v1,v2,v3,v4);
		
		ofPopMatrix();
	}
}