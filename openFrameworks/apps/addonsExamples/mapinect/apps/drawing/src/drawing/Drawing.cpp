#include "Drawing.h"

#include "Globals.h"
#include "ofGraphicsUtils.h"

namespace drawing {
	
	//--------------------------------------------------------------
	Drawing::Drawing()
		: backColor(kRGBWhite), foreColor(kRGBBlack)
	{
	}

	//--------------------------------------------------------------
	Drawing::~Drawing()
	{
	}

	//--------------------------------------------------------------
	void Drawing::setup()
	{
	}

	//--------------------------------------------------------------
	void Drawing::exit()
	{
	}

	//--------------------------------------------------------------
	void Drawing::debugDraw()
	{
	}

	//--------------------------------------------------------------
	void Drawing::draw()
	{
	}


	//--------------------------------------------------------------
	void Drawing::update()
	{
	}

	//--------------------------------------------------------------
	void Drawing::keyPressed(int key)
	{
	}

	//--------------------------------------------------------------
	void Drawing::mouseMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void Drawing::mouseDragged(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void Drawing::mousePressed(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void Drawing::mouseReleased(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void Drawing::windowResized(int w, int h)
	{
	}

	//--------------------------------------------------------------
	void Drawing::objectDetected(const IObjectPtr& object)
	{
		map<int, map<int, Canvas*> >::iterator ob = canvas.find(object->getId());
		assert(ob == canvas.end());
		if (ob == canvas.end())
		{
			map<int, Canvas*> objectCanvas;
			for (vector<IPolygonPtr>::const_iterator p = object->getPolygons().begin(); p != object->getPolygons().end(); ++p)
			{
				objectCanvas[(*p)->getId()] = new Canvas(txManager, *p, backColor, foreColor);
			}
			canvas[object->getId()] = objectCanvas;
		}
	}

	//--------------------------------------------------------------
	void Drawing::objectUpdated(const IObjectPtr& object)
	{
	}

	//--------------------------------------------------------------
	void Drawing::objectLost(const IObjectPtr& object)
	{
	}

	//--------------------------------------------------------------
	void Drawing::objectMoved(const IObjectPtr& object, const DataMovement& movement)
	{
	}
	
	//--------------------------------------------------------------
	void Drawing::objectTouched(const IObjectPtr& object, const DataTouch& touchPoint)
	{
		map<int, map<int, Canvas*> >::iterator ob = canvas.find(object->getId());
		assert(ob != canvas.end());
		if (ob != canvas.end())
		{
			map<int, Canvas*>::iterator p = ob->second.find(touchPoint.getPolygon()->getId());
			assert(p != ob->second.end());
			if (p != ob->second.end())
			{
				p->second->touchEvent(touchPoint);
			}
		}
	}
}
