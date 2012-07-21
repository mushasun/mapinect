#include "Pown.h"

#include "ofGraphicsUtils.h"
#include "Timer.h"

namespace pown
{
	const float kEmitTime = 1.0f;
	static Timer timer;

	static int emisor = -1;

	Pown::Pown()
	{
	}

	Pown::~Pown()
	{
		for (map<int, Box*>::iterator i = boxes.begin(); i != boxes.end(); i++) {
			delete i->second;
		}
		boxes.clear();
	}

	void Pown::setup()
	{
	}

	void Pown::draw()
	{
		if (floor.get() != NULL)
		{
			ofSetColor(kRGBWhite);
			for (vector<IPolygonPtr>::const_iterator p = floor->getPolygons().begin(); p != floor->getPolygons().end(); ++p)
			{
				ofDrawQuad((*p)->getMathModel().getVertexs());
			}
		}
		for (set<Spot*>::const_iterator spot = spots.begin(); spot != spots.end(); spot++)
			(*spot)->draw();
		for (set<Bolt*>::const_iterator bolt = bolts.begin(); bolt != bolts.end(); bolt++)
			(*bolt)->draw();
		for (map<int, Box*>::iterator iter = boxes.begin(); iter != boxes.end(); iter++)
			(iter->second)->draw();
	}

	void Pown::update(float elapsedTime)
	{
		if (emisor > 0)
		{
			map<int, Box*>::iterator boxIterator = boxes.find(emisor);
			if (boxIterator != boxes.end())
			{
				Box* box = boxIterator->second;
				timer.stop();
				if (timer.getElapsedSeconds() >= kEmitTime)
				{
					ofColor color(255, 0, 0);
					ofVec3f boltCenter = floor->getPolygons()[0]->getMathModel().getPlane().project(box->getCenter());
					Bolt* bolt = new Bolt(color, boltCenter, ofVec3f(0.4f, 0, 0));
					bolts.insert(bolt);
				}
			}
		}
		for (set<Spot*>::const_iterator spot = spots.begin(); spot != spots.end(); spot++)
			(*spot)->update(elapsedTime);
		for (set<Bolt*>::const_iterator bolt = bolts.begin(); bolt != bolts.end(); bolt++)
			(*bolt)->update(elapsedTime);
		for (map<int, Box*>::iterator iter = boxes.begin(); iter != boxes.end(); iter++)
			(iter->second)->update(elapsedTime);
		
		for (set<Bolt*>::iterator bolt = bolts.begin(); bolt != bolts.end();)
		{
			if (!(*bolt)->isAlive())
				bolts.erase(bolt++);
			else
				++bolt;
		}
	}

	void Pown::objectDetected(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			floor = object;
		}
		else
		{
			if (boxes.find(object->getId()) == boxes.end())
			{
				ofColor color(ofRandomf() * 255.0f, ofRandomf() * 255.0f, ofRandomf() * 255.0f);
				boxes[object->getId()] = new Box(object, color);
				if (emisor < 0)
				{
					emisor = object->getId();
					timer.start();
				}
			}
		}
	}
		
	void Pown::objectUpdated(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			floor = object;
		}
		else
		{
			map<int, Box*>::iterator b = boxes.find(object->getId());
			if (b != boxes.end())
				b->second->updateModelObject(object);
		}
	}

	void Pown::objectLost(const IObjectPtr& object)
	{
		if (object->getId() != TABLE_ID)
		{
			if (boxes.find(object->getId()) != boxes.end())
			{
				boxes.erase(object->getId());
			}
		}
	}

	void Pown::objectMoved(const IObjectPtr& object, const DataMovement& movement)
	{
		if (object->getId() != TABLE_ID)
		{
			map<int, Box*>::iterator b = boxes.find(object->getId());
			if (b != boxes.end())
			{
				b->second->updateModelObject(object);
			}
		}
	}
	
	void Pown::objectTouched(const IObjectPtr& object, const DataTouch& touchPoint)
	{
	}

	void Pown::buttonPressed(const IButtonPtr& btn)
	{
	}
	void Pown::buttonReleased(const IButtonPtr& btn)
	{
	}
}
