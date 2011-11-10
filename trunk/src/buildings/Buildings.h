#ifndef BUILDINGS_H__
#define BUILDINGS_H__

#include "ofImage.h"
#include "ofxFenster.h"
#include "ofxVec3f.h"

namespace mapinect {

	class buildings {
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

		bool bTexApplied;
		GLuint textureID;
		GLuint streetT;
		GLuint camino;

		float screenFov;		//25.40f//28.04f;
		float aspect;			//1.36f//1.35f;
		float nearPlane;
		float zAngle;

		std::vector<float> porcentajes;

		float transXAT; //-11.0;//-45.0;			//-59.0;		
		float transYAT; //76.0;//-6.0;		

		// Coordenadas 3D del proyector
		float xProj;//-20.0f;		
		float yProj;//33.0f;		// 364 mm arriba del Kinect
		float zProj;//0.0f;			// 720 o 900 mm de distancia (z) del Kinect
		

		GLuint loadImageTexture(char* imgFile); 
//		GLuint loadImageTexture(int& imgIndex); 

		ofxFenster* fenster;

	};
}

#endif	// BUILDINGS_H__
