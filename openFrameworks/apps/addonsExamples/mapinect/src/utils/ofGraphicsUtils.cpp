#include "ofGraphicsUtils.h"

#include <assert.h>

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

void ofDrawQuad(const vector<ofVec3f>& vertexs)
{
	assert(vertexs.size() == 4);
	ofDrawQuadTextured(vertexs[0], vertexs[1], vertexs[2], vertexs[3]);
}

void ofDrawQuadTextured(const vector<ofVec3f>& vertexs, const vector<ofVec2f>& texCoords)
{
	assert(vertexs.size() == 4 && texCoords.size() == 4);
	ofDrawQuadTextured(vertexs[0], vertexs[1], vertexs[2], vertexs[3],
						texCoords[0].x, texCoords[0].y, texCoords[1].x, texCoords[1].y,
						texCoords[2].x, texCoords[2].y, texCoords[3].x, texCoords[3].y);
}

vector<ofVec2f> ofTexCoordsFor(ofBaseDraws& d)
{
	vector<ofVec2f> result;
	result.push_back(ofVec2f(0, 0));
	result.push_back(ofVec2f(d.getWidth(), 0));
	result.push_back(ofVec2f(d.getWidth(), d.getHeight()));
	result.push_back(ofVec2f(0, d.getHeight()));
	
	return result;
}
