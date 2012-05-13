#include "ColorBalancing.h"

#include "Globals.h"

namespace colorbalancing {
#define KWIDTH		640
#define KHEIGHT		480

	//--------------------------------------------------------------
	void ColorBalancing::setup() {

	}

	//--------------------------------------------------------------
	void ColorBalancing::exit() {

	}

	//--------------------------------------------------------------
	void ColorBalancing::draw() {

	}

	ofColor camera[KWIDTH * KHEIGHT];
	unsigned char rgb[KWIDTH * KHEIGHT * 3];
	//--------------------------------------------------------------
	void ColorBalancing::debugDraw() {
		float maxColor = 0.0f;
		for (int i = 0; i < KWIDTH; i++) {
			for (int j = 0; j < KHEIGHT; j++) {
				ofColor c = gKinect->getColorAt(i, j);
				float r = (float)c.r / 255.0f;
				float g = (float)c.g / 255.0f;
				float b = (float)c.b / 255.0f;
				camera[j * KWIDTH + i] = c;
				maxColor = max(max(r, g), max(b, maxColor));
			}
		}
		if (maxColor < 0.001) {
			return;
		}

		// now we balance the channels
		for (int j = 0; j < KHEIGHT; j++) {
			for (int i = 0; i < KWIDTH; i++) {
				ofColor c = camera[j * KWIDTH + i];
				c.r /= maxColor;
				c.g /= maxColor;
				c.b /= maxColor;
				int ix = (j * KWIDTH + i) * 3;
				rgb[ix] = c.r * 255.0f;
				rgb[ix + 1] = c.g * 255.0f;
				rgb[ix + 2] = c.b * 255.0f;
			}
		}

		ofImage image;
		image.setFromPixels(rgb, KWIDTH, KHEIGHT, OF_IMAGE_COLOR);
		image.draw(0, 0);

		gKinect->draw(KWIDTH, 0);
	}

	//--------------------------------------------------------------
	void ColorBalancing::update() {
		
	}

	//--------------------------------------------------------------
	void ColorBalancing::keyPressed(int key) {

	}

	//--------------------------------------------------------------
	void ColorBalancing::mouseMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void ColorBalancing::mouseDragged(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void ColorBalancing::mousePressed(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void ColorBalancing::mouseReleased(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void ColorBalancing::windowResized(int w, int h)
	{
	}

}
