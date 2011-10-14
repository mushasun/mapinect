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

#endif	// OFGRAPHICS_UTILS_H__
