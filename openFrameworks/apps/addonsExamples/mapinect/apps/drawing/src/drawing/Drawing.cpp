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
	void Drawing::update(float elapsedTime)
	{
		static float redrawTime = 0;
		const float redrawTimeLimit = 500.0f;
		redrawTime += elapsedTime;
		if (redrawTime > redrawTimeLimit)
		{
			for (map<int, map<int, Canvas*> >::iterator ob = canvas.begin(); ob != canvas.end(); ++ob)
			{
				for (map<int, Canvas*>::iterator p = ob->second.begin(); p != ob->second.end(); ++p)
				{
					p->second->redrawIfNecessary();
				}
			}
			redrawTime = 0.0f;
		}
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
