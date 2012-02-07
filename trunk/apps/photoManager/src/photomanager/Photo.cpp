#include "Photo.h"

using namespace mapinect;
namespace photo {
	Photo::Photo()
	{
		this->position = ofxVec3f(0,0,0);
		this->normal = ofxVec3f(0,1,0);
		this->scale = ofxVec3f(1,1,1);
		this->rotation = 0;
	}
	void Photo::draw(const ITxManager* txManager)
	{
		ofPushMatrix();

		////Hallo la rotación http://www.gamedev.net/topic/472246-rotation-matrix-between-two-vectors/
		ofxVec3f w (normal);
		w.cross(ofxVec3f(0,0,1));

		float angle = (asin(w.length())* 180) / PI;
		w.normalize();
		

		ofTranslate(position.x,position.y,position.z);
		ofRotate(rotation,normal.x,normal.y,normal.z);
		ofScale(scale.x,scale.y,scale.z);
		ofRotate(-angle,1,0,0);

		// Bind Texture
		txManager->bindTexture(texture);

		
		ofxVec3f v1, v2,v3,v4;
		v1 = ofxVec3f();
		v2 = v1;
		v2.x += width;
		v3 = v2;
		v3.y += height;
		v4 = ofxVec3f();
		v4.y += height;
		
		

		txManager->enableTextures();
		txManager->bindTexture(texture);
		ofDrawQuadTextured(v1,v2,v3,v4);
		txManager->disableTextures();

		
		ofPopMatrix();
	}
}