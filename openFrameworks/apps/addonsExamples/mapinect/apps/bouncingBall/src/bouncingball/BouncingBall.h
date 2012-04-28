#ifndef BOUNCING_BALL_H__
#define BOUNCING_BALL_H__

#include "IApplication.h"

#include "ofVecUtils.h"
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

		virtual void debugDraw();
	
	private:
		bool tableSetted;
		Tejo ball;
		std::vector<Segment3D> tableSegment3Ds;
		std::vector<Segment3D> segments;

		ofVec3f tableNormal;
		ofVec3f tableCenter;
		float y_angle;
		vector<ofVec3f> closerToTable(vector<ofVec3f> lst);
		void getOrderedPoints(vector<ofVec3f> vecsproj,ofVec3f& left,ofVec3f& top,ofVec3f& right,ofVec3f& bottom,ofVec3f& centroid);
		BObject* getBObject(int id);
		void clearUnvisitedObjects();
		vector<BObject*> bobjects;
		BObject* table;
	};
}

#endif	// BOUNCING_BALL_H__