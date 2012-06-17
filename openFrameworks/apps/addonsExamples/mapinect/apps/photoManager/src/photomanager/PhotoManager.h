#ifndef PHOTO_MANAGER_H__
#define PHOTO_MANAGER_H__

#include "IApplication.h"

#include "ofVecUtils.h"
#include "Photo.h"
#include "Grab.h"

namespace photo {
	class PhotoManager : public IApplication {
	public:
		virtual void setup();
		virtual void update();
		virtual void draw();
		virtual void exit();

		virtual void keyPressed(int key);
		virtual void keyReleased(int key);
		virtual void windowMoved(int x, int y);
		virtual void mouseMoved(int x, int y);
		virtual void mouseDragged(int x, int y, int button);
		virtual void mousePressed(int x, int y, int button);
		virtual void mouseReleased(int x, int y, int button);
		virtual void dragEvent(ofDragInfo info);

		virtual void debugDraw();

		virtual void objectDetected(const IObjectPtr&);
		virtual void objectUpdated(const IObjectPtr&);
		virtual void objectLost(const IObjectPtr&);
		virtual void objectMoved(const IObjectPtr&, const DataMovement&);
		virtual void objectTouched(const IObjectPtr&, const DataTouch&);

		static GLuint	tableTexture;
	private:
		bool	isHandInPhoto(PCHand* hand,Photo* photo);

		list<Photo*> photos;
		list<Grab> grabs;
		PCPolygon*		table;
	};
}

#endif	// PHOTO_MANAGER_H__
