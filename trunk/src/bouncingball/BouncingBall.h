#ifndef BouncingBall_H__
#define BouncingBall_H__
#include "ofxVecUtils.h"
#include "Tejo.h"
#include "Segment3D.h"

namespace mapinect {
	class BouncingBall {
	public:
		virtual void setup();
		virtual void update();
		virtual void draw();

		virtual void keyPressed(int key);
		virtual void mouseMoved(int x, int y );
		virtual void mouseDragged(int x, int y, int button);
		virtual void mousePressed(int x, int y, int button);
		virtual void mouseReleased(int x, int y, int button);
		virtual void windowResized(int w, int h);
	private:
		float screenFov;		//28.04f;
		float aspect;			//1.35f;
		float nearPlane;
		float zAngle;

		float transXAT;			//-59.0;		
		float transYAT;		

		// Coordenadas 3D del proyector
		float xProj;		
		float yProj;		// 364 mm arriba del Kinect
		float zProj;			// 720 o 900 mm de distancia (z) del Kinect

		float projectionMatrix[16];

		// Dibujar Quad
		ofxVec3f vA,vB,vC,vD;

		ofxVec3f carasADibujar[12][4];
		bool tableSetted;
		Tejo ball;
		std::vector<Segment3D> tableSegment3Ds;
		std::vector<Segment3D> segments;

		ofxVec3f tableNormal;
		ofxVec3f tableCenter;
		float y_angle;
		vector<ofxVec3f> closerToTable(vector<ofxVec3f> lst);
		void getOrderedPoints(vector<ofxVec3f> vecsproj,ofxVec3f& left,ofxVec3f& top,ofxVec3f& right,ofxVec3f& bottom,ofxVec3f& centroid);
	};
}

#endif	// BouncingBall_H__
