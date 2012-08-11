#include "Pown.h"

#include "ofGraphicsUtils.h"
#include "ofLight.h"

#include "PownConstants.h"
#include "SoundManager.h"
#include "Timer.h"

namespace pown
{

	static ofLight ambientLight;

	Pown::Pown()
		: emisor(-1), floor(NULL)
	{
		ambientLight.setDirectional();
		ambientLight.setAmbientColor(kRGBRed);
		ambientLight.setDiffuseColor(kRGBWhite);
		ambientLight.setSpecularColor(kRGBGreen);
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
		PownConstants::LoadPownConstants();
		SoundManager::setup();
		Spot::setup();
	}

	void Pown::draw()
	{
		ofEnableAlphaBlending();
		//ofEnableLighting();

		//ambientLight.enable();
		ambientLight.setPosition(ofVec3f(0, -0.3, 0));
		
		if (floor != NULL)
			floor->draw();
		for (set<Spot*>::const_iterator spot = spots.begin(); spot != spots.end(); spot++)
			(*spot)->draw();
		for (set<Bolt*>::const_iterator bolt = bolts.begin(); bolt != bolts.end(); bolt++)
			(*bolt)->draw();
		for (map<int, Box*>::iterator box = boxes.begin(); box != boxes.end(); box++)
			(box->second)->draw();
		
		//ambientLight.disable();
		
		//ofDisableLighting();
		ofDisableAlphaBlending();
	}

	void Pown::testCollisions()
	{
		// test spots with boxes
		for (set<Spot*>::iterator spot = spots.begin(); spot != spots.end(); spot++)
			for (map<int, Box*>::iterator box = boxes.begin(); box != boxes.end(); box++)
				if ((*spot)->testHit(box->second))
					(*spot)->setBox(box->second);

		// test bolts with boxes
		for (map<int, Box*>::iterator box = boxes.begin(); box != boxes.end(); box++)
			if (box->first != emisor)
				for (set<Bolt*>::iterator bolt = bolts.begin(); bolt != bolts.end(); bolt++)
					if ((*bolt)->isAlive() && box->second->testHit(*bolt))
						box->second->absorbBolt(*bolt);

		// test bolts with floor
		if (floor != NULL)
			for (set<Bolt*>::iterator bolt = bolts.begin(); bolt != bolts.end(); bolt++)
				if ((*bolt)->isAlive() && floor->testHit(*bolt))
					floor->absorbBolt(*bolt);
	}

	void Pown::handleBoltEmision(float elapsedTime)
	{
		map<int, Box*>::iterator boxIterator = boxes.find(emisor);
		if (boxIterator != boxes.end())
		{
			Box* box = boxIterator->second;
			static float boltEmisorTimer = 0;
			boltEmisorTimer += elapsedTime;
			if (boltEmisorTimer >= PownConstants::EMIT_TIME)
			{
				boltEmisorTimer -= PownConstants::EMIT_TIME;

				const int boltsPeriod = 8;
				const int boltSubPeriod = 4; 
				static int delay = 0;
				float mismatch = (float)delay * TWO_PI / ((float)boltSubPeriod * (float)boltsPeriod);
				delay = (delay + 1) % boltSubPeriod;

				for (int i = 0; i < boltsPeriod; i++)
				{
					ofColor color(ofRandomColor());
				
					ofVec3f boltCenter = floor->getObject()->getPolygons()[0]->getMathModel().getPlane().project(box->getCenter());
					boltCenter.y -= 0.001f;

					ofVec3f boltSpeed = ofVec3f(1, 0, 0);
					boltSpeed *= PownConstants::BOLT_INITIAL_SPEED;
					boltSpeed.rotateRad(0, (float)i * TWO_PI / (float)boltsPeriod + mismatch, 0);

					Bolt* bolt = new Bolt(color, boltCenter, boltSpeed);
					// Ensure the bolt takes up the missing delay
					bolt->update(boltEmisorTimer);
					bolts.insert(bolt);
				}
			}
		}
	}

	void Pown::update(float elapsedTime)
	{
		SoundManager::update(elapsedTime);

		// update all existing objects
		if (floor != NULL)
			floor->update(elapsedTime);
		for (set<Spot*>::const_iterator spot = spots.begin(); spot != spots.end(); spot++)
			(*spot)->update(elapsedTime);
		for (set<Bolt*>::const_iterator bolt = bolts.begin(); bolt != bolts.end(); bolt++)
			(*bolt)->update(elapsedTime);
		for (map<int, Box*>::iterator box = boxes.begin(); box != boxes.end(); box++)
			(box->second)->update(elapsedTime);
		
		testCollisions();

		// emit bolts
		handleBoltEmision(elapsedTime);

		// remove dead bolts
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
			floor = new Floor(object, kRGBWhite);
		}
		else
		{
			if (boxes.find(object->getId()) == boxes.end())
			{
				ofColor color(ofRandomf() * 255.0f, ofRandomf() * 255.0f, ofRandomf() * 255.0f);
				Box* box = new Box(object, color);
				boxes[object->getId()] = box;
				ofVec3f spotCenter = floor->getObject()->getPolygons()[0]->getMathModel().getPlane().project(box->getCenter());
				spotCenter.y -= 0.001f;
				Spot* spot = new Spot(spotCenter);
				spots.insert(spot);
				if (emisor < 0)
				{
					emisor = object->getId();
				}
			}
		}
	}
		
	void Pown::objectUpdated(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			floor->updateModelObject(object);
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
			if (emisor == object->getId())
			{
				emisor = -1;
				if (boxes.begin() != boxes.end())
				{
					emisor = boxes.begin()->first;
				}
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
