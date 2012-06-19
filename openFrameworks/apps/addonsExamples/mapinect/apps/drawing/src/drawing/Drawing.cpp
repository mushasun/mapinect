#include "Drawing.h"

#include "ofGraphicsUtils.h"

namespace drawing {
	
	//--------------------------------------------------------------
	Drawing::Drawing()
		: backColor(kRGBWhite), foreColor(kRGBRed)
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
		for (map<int, map<int, Canvas*> >::iterator ob = canvas.begin(); ob != canvas.end(); ++ob)
		{
			for (map<int, Canvas*>::iterator p = ob->second.begin(); p != ob->second.end(); ++p)
			{
				p->second->draw();
			}
		}
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
	void Drawing::keyReleased(int key)
	{
	}

	//--------------------------------------------------------------
	void Drawing::windowMoved(int x, int y)
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
	void Drawing::dragEvent(ofDragInfo info)
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
				objectCanvas[(*p)->getName()] = new Canvas(*p, backColor, foreColor);
				backColor = ofColor(rand() % 255, rand() % 255, rand() % 255);
			}
			canvas[object->getId()] = objectCanvas;
		}
	}

	//--------------------------------------------------------------
	void Drawing::objectUpdated(const IObjectPtr& object)
	{
		map<int, map<int, Canvas*> >::iterator ob = canvas.find(object->getId());
		if (ob != canvas.end())
		{
			for (vector<IPolygonPtr>::const_iterator p = object->getPolygons().begin(); p != object->getPolygons().end(); ++p)
			{
				map<int, Canvas*>::iterator c = ob->second.find((*p)->getName());
				if (c != ob->second.end())
				{
					c->second->update(*p);
				}
			}
		}
	}

	//--------------------------------------------------------------
	void Drawing::objectLost(const IObjectPtr& object)
	{
		map<int, map<int, Canvas*> >::iterator ob = canvas.find(object->getId());
		if (ob != canvas.end())
		{
			for (map<int, Canvas*>::iterator c = ob->second.begin(); c != ob->second.end(); ++c)
			{
				delete c->second;
			}
			canvas.erase(object->getId());
		}
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
			map<int, Canvas*>::iterator p = ob->second.find(touchPoint.getPolygon()->getName());
			assert(p != ob->second.end());
			if (p != ob->second.end())
			{
				p->second->touchEvent(touchPoint);
			}
		}

	}
}
