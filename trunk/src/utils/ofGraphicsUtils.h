#ifndef OFGRAPHICS_UTILS_H__
#define OFGRAPHICS_UTILS_H__

#include "ofGraphics.h"

#define kRGBWhite		0xFFFFFF
#define kRGBBlack		0x000000
#define kRGBRed			0xFF0000
#define kRGBGreen		0x00FF00
#define kRGBBlue		0x0000FF

inline void ofResetColor() {
	ofSetColor(kRGBWhite);
}

void ofDrawQuadTextured(const ofPoint& vA, const ofPoint& vB, const ofPoint& vC, const ofPoint& vD,
	float sA = 0, float tA = 0, float sB = 1, float tB = 0, float sC = 1, float tC = 1, float sD = 0, float tD = 1);

#endif	// OFGRAPHICS_UTILS_H__
