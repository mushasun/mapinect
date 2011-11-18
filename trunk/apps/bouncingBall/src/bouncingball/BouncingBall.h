#ifndef BOUNCING_BALL_H__
#define BOUNCING_BALL_H__

#include "IApplication.h"

#include "ofxVecUtils.h"
#include "Tejo.h"
#include "Segment3D.h"
#include "BObject.h"

namespace bouncing {
	class BouncingBall : public IApplication {
	public:
		virtual void setup();
		virtual void exit();
		virtual void update();
		virtual void draw();

		virtual void keyPressed(int key);
		virtual void mouseMoved(int x, int y );
		virtual void mouseDragged(int x, int y, int button);
		virtual void mousePressed(int x, int y, int button);
		virtual void mouseReleased(int x, int y, int button);
		virtual void windowResized(int w, int h);
	
	private:
		bool tableSetted;
		Tejo ball;
		std::vector<Segment3D> tableSegment3Ds;
		std::vector<Segment3D> segments;

		ofxVec3f tableNormal;
		ofxVec3f tableCenter;
		float y_angle;
		vector<ofxVec3f> closerToTable(vector<ofxVec3f> lst);
		void getOrderedPoints(vector<ofxVec3f> vecsproj,ofxVec3f& left,ofxVec3f& top,ofxVec3f& right,ofxVec3f& bottom,ofxVec3f& centroid);
		BObject* getBObject(int id);
		void clearUnvisitedObjects();
		vector<BObject*> bobjects;
		BObject* table;
	};
}

#endif	// BOUNCING_BALL_H__
