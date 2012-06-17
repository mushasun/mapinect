#ifndef OFGRAPHICS_UTILS_H__
#define OFGRAPHICS_UTILS_H__

#include "ofGraphics.h"

enum BasicColors
{
	kRGBWhite			= 0xFFFFFF,
	kRGBBlack			= 0x000000,
	kRGBLightGray		= 0xC0C0C0,
	kRGBDarkGray		= 0x808080,
	kRGBRed				= 0xFF0000,
	kRGBGreen			= 0x00FF00,
	kRGBBlue			= 0x0000FF,
	kRGBCyan			= 0x00FFFF,
	kRGBYellow			= 0xFFFF00,
	kRGBMagenta			= 0xFF00FF,
	kRGBDarkGreen		= 0x008000,
	kRGBOrange			= 0xFF8000,
	kRGBBrown			= 0x804000,
	kRGBSkyBlue			= 0x0080FF,
	kRGBViolet			= 0x800080,
	kRGBPurple			= 0x400040,
	kRGBBeige			= 0x808000
};

inline void ofResetColor() {
	ofSetHexColor(kRGBWhite);
}

void ofDrawQuadTextured(const ofPoint& vA, const ofPoint& vB, const ofPoint& vC, const ofPoint& vD,
	float sA = 0, float tA = 0, float sB = 1, float tB = 0, float sC = 1, float tC = 1, float sD = 0, float tD = 1);
void ofDrawQuadTextured(const vector<ofVec3f>& vertexs, const vector<ofVec2f>& texCoords);
vector<ofVec2f> ofTexCoordsFor(ofBaseDraws& d);

#endif	// OFGRAPHICS_UTILS_H__
