#include "Calibration.h"

#include "Globals.h"

namespace calibration {

	const int s = 4;

	//--------------------------------------------------------------
	void Calibration::setup()
	{
		pattern.loadImage("pattern.bmp");
	}

	//--------------------------------------------------------------
	void Calibration::exit() {

	}

	//--------------------------------------------------------------
	void Calibration::draw()
	{
		static ofVec3f wc[KINECT_DEFAULT_WIDTH][KINECT_DEFAULT_HEIGHT];
		for (int i = 0; i < KINECT_DEFAULT_WIDTH; i += s)
			for (int j = 0; j < KINECT_DEFAULT_HEIGHT; j += s)
				wc[i][j] = gKinect->getWorldCoordinateFor(i, j);

		glBegin(GL_QUADS);
		for (int i = 0, ci = 0; i < KINECT_DEFAULT_WIDTH; i += s, ci++)
		{
			float fi = (float)i;
			for (int j = 0, cj = 0; j < KINECT_DEFAULT_HEIGHT; j += s, cj++)
			{
				float fj = (float)j;
				ofSetColor(pattern.getColor(ci % pattern.width, cj % pattern.height));

				float qs = 0.005;
				ofVec3f w = wc[i][j];
				if (w.z > 0)
				{
					ofVec3f wr = w + ofVec3f(qs, 0, 0);
					ofVec3f wrb = w + ofVec3f(qs, qs, 0);
					ofVec3f wb = w + ofVec3f(0, qs, 0);
					glVertex3f(w.x, w.y, w.z);
					glVertex3f(wr.x, wr.y, wr.z);
					glVertex3f(wrb.x, wrb.y, wrb.z);
					glVertex3f(wb.x, wb.y, wb.z);
				}
				//if (w.z > 0)
				//{
				//	ofVec3f wr = (i+s < KINECT_DEFAULT_WIDTH && wc[i+s][j].z > 0) ? wc[i+s][j] : w + ofVec3f(qs, 0, 0);
				//	ofVec3f wrb = (i+s < KINECT_DEFAULT_WIDTH && j+s < KINECT_DEFAULT_HEIGHT && wc[i+s][j+s].z > 0) ? wc[i+s][j+s] : w + ofVec3f(qs, qs, 0);
				//	ofVec3f wb = (j+s < KINECT_DEFAULT_HEIGHT && wc[i][j+s].z > 0) ? wc[i][j+s] : w + ofVec3f(0, qs, 0);
				//	glVertex3f(w.x, w.y, w.z);
				//	glVertex3f(wr.x, wr.y, wr.z);
				//	glVertex3f(wrb.x, wrb.y, wrb.z);
				//	glVertex3f(wb.x, wb.y, wb.z);
				//}
			}
		}
		glEnd();
	}

	//--------------------------------------------------------------
	void Calibration::debugDraw()
	{
		glBegin(GL_QUADS);
		for (int i = 0, ci = 0; i < KINECT_DEFAULT_WIDTH; i += s, ci++)
		{
			float fi = (float)i;
			for (int j = 0, cj = 0; j < KINECT_DEFAULT_HEIGHT; j += s, cj++)
			{
				float fj = (float)j;
				ofSetColor(pattern.getColor(ci % pattern.width, cj % pattern.height));

				float qs = (float)(s / 2);
				glVertex3f(fi, fj, 0);
				glVertex3f(fi + qs, fj, 0);
				glVertex3f(fi + qs, fj + qs, 0);
				glVertex3f(fi, fj + qs, 0);
			}
		}
		glEnd();
	}

	//--------------------------------------------------------------
	void Calibration::update() {
		
	}

	//--------------------------------------------------------------
	void Calibration::keyPressed(int key) {

	}

	//--------------------------------------------------------------
	void Calibration::mouseMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void Calibration::mouseDragged(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void Calibration::mousePressed(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void Calibration::mouseReleased(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void Calibration::windowResized(int w, int h)
	{
	}

}
