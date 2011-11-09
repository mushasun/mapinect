#ifndef VM_H__
#define VM_H__

#include "ofImage.h"
#include "ofxFenster.h"

namespace mapinect {

	class VM {
	public:
		virtual void setup(ofxFenster* f);
		virtual void update();
		virtual void draw();

		virtual void keyPressed(int key);
		virtual void mouseMoved(int x, int y );
		virtual void mouseDragged(int x, int y, int button);
		virtual void mousePressed(int x, int y, int button);
		virtual void mouseReleased(int x, int y, int button);
		virtual void windowResized(int w, int h);

		ofImage img;
		bool bImgLoaded;
		int imgIndex;
		char* imgFilename;
		// ofTexture tex;
		static unsigned char* imgPixels;
		static GLuint textureID;
		bool bTexApplied;

		GLuint loadImageTexture(char* imgFile); 
//		GLuint loadImageTexture(int& imgIndex); 

		ofxFenster* fenster;

	};
}

#endif	// VM_H__
