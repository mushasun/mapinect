#ifndef BOUNCING_BALL_H__
#define BOUNCING_BALL_H__

#include "IApplication.h"

#include "ofxVecUtils.h"
#include "Photo.h"
#include "Grab.h"

namespace photo {
	class PhotoManager : public IApplication {
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
		static GLuint	tableTexture;
	private:
		bool	isHandInPhoto(PCHand* hand,Photo* photo);

		list<Photo*> photos;
		list<Grab> grabs;
		PCPolygon*		table;
	};
}

#endif	// BOUNCING_BALL_H__
