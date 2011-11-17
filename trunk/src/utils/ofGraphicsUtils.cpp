#include "ofGraphicsUtils.h"

void ofDrawQuadTextured(const ofPoint& vA, const ofPoint& vB, const ofPoint& vC, const ofPoint& vD,
	float sA, float tA, float sB, float tB, float sC, float tC, float sD, float tD) {
	
	glBegin(GL_QUADS);      
		glTexCoord2f(sA, tA);
		glVertex3f(vA.x, vA.y, vA.z); 
		glTexCoord2f(sB, tB);
		glVertex3f(vB.x, vB.y, vB.z);
		glTexCoord2f(sC, tC);
		glVertex3f(vC.x, vC.y, vC.z);
		glTexCoord2f(sD, tD);
		glVertex3f(vD.x, vD.y, vD.z);
	glEnd();
}

