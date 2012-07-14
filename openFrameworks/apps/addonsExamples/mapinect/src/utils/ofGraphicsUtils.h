#ifndef OFGRAPHICS_UTILS_H__
#define OFGRAPHICS_UTILS_H__

#include "ofGraphics.h"

const ofColor kRGBWhite			(255, 255, 255);
const ofColor kRGBBlack			(0, 0, 0);
const ofColor kRGBLightGray		(192, 192, 192);
const ofColor kRGBDarkGray		(128, 128, 128);
const ofColor kRGBRed			(255, 0, 0);
const ofColor kRGBGreen			(0, 255, 0);
const ofColor kRGBBlue			(0, 0, 255);
const ofColor kRGBCyan			(0, 255, 255);
const ofColor kRGBYellow		(255, 255, 0);
const ofColor kRGBMagenta		(255, 0, 255);
const ofColor kRGBDarkGreen		(0, 128, 0);
const ofColor kRGBOrange		(255, 128, 0);
const ofColor kRGBBrown			(128, 64, 0);
const ofColor kRGBSkyBlue		(0, 128, 255);
const ofColor kRGBViolet		(128, 0, 128);
const ofColor kRGBPurple		(64, 0, 64);
const ofColor kRGBBeige			(128, 128, 0);

inline void ofResetColor() {
	ofSetColor(kRGBWhite);
}

void ofDrawQuadTextured(const ofPoint& vA, const ofPoint& vB, const ofPoint& vC, const ofPoint& vD,
	float sA = 0, float tA = 0, float sB = 1, float tB = 0, float sC = 1, float tC = 1, float sD = 0, float tD = 1);
void ofDrawQuad(const vector<ofVec3f>& vertexs);
void ofDrawQuadTextured(const vector<ofVec3f>& vertexs, const vector<ofVec2f>& texCoords);
vector<ofVec2f> ofTexCoordsFor(ofBaseDraws& d);

#endif	// OFGRAPHICS_UTILS_H__
