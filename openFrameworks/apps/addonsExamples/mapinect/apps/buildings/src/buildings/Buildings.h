#ifndef BUILDINGS_H__
#define BUILDINGS_H__

#include "IApplication.h"

#include "Floor.h"
#include "Building.h"
#include <map>

namespace buildings {

	class Buildings : public IApplication {
	public:
		Buildings();
		virtual ~Buildings();

		virtual void setup();
		virtual void update();
		virtual void draw();
		virtual void exit();

		virtual void keyPressed(int key);
		virtual void mouseMoved(int x, int y );
		virtual void mouseDragged(int x, int y, int button);
		virtual void mousePressed(int x, int y, int button);
		virtual void mouseReleased(int x, int y, int button);
		virtual void windowResized(int w, int h);

		virtual void debugDraw();

		virtual void objectDetected(const IObjectPtr&);
		virtual void objectUpdated(const IObjectPtr&);
		virtual void objectLost(const IObjectPtr&);
		virtual void objectMoved(const IObjectPtr&, const DataMovement&);
		virtual void objectTouched(const IObjectPtr&, const DataTouch&);

		static GLuint	videoTexture;
		static GLuint	videoTexture2;

	private:
		std::map<int, Building*>	buildings;
		std::map<int, DataTouch>	touchPoints;
		Floor*						floor;
	};
}

#endif	// BUILDINGS_H__
