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
		: brickManager(NULL)
	{
		floor.reset();
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
		
		if (brickManager != NULL)
			brickManager->draw();
		for (set<Spot*>::const_iterator spot = spots.begin(); spot != spots.end(); spot++)
			(*spot)->draw();
		for (map<int, Box*>::iterator box = boxes.begin(); box != boxes.end(); box++)
			(box->second)->draw();
		
		//ambientLight.disable();
		
		//ofDisableLighting();
		ofDisableAlphaBlending();
	}

	void Pown::handleCreateSpot(float elapsedTime)
	{
		static float spotTimer = 0.0f;
		spotTimer += elapsedTime;
		if (spotTimer >= PownConstants::SPOT_SEED_TIME)
		{
			spotTimer -= PownConstants::SPOT_SEED_TIME;
			/*
			ofVec3f spotCenter = floor->getMathModel().getPlane().project(box->getCenter());
			spotCenter.y -= 0.001f;
			Spot* spot = new Spot(spotCenter);
			spots.insert(spot);
			*/
		}
	}

	void Pown::testCollisions()
	{
		// test spots with boxes
		for (set<Spot*>::iterator spot = spots.begin(); spot != spots.end(); spot++)
			for (map<int, Box*>::iterator box = boxes.begin(); box != boxes.end(); box++)
				if ((*spot)->testHit(box->second))
					;
	}

	void Pown::updateBeat(float elapsedTime)
	{
		if (brickManager != NULL)
		{
			static float boltEmisorTimer = 0;
			boltEmisorTimer += elapsedTime;
			if (boltEmisorTimer >= PownConstants::BEAT_TIME)
			{
				boltEmisorTimer -= PownConstants::BEAT_TIME;

				brickManager->doBeat(boxes);

				SoundManager::beat();
			}
		}
	}

	void Pown::update(float elapsedTime)
	{
		// update all existing objects
		if (brickManager != NULL)
			brickManager->update(elapsedTime);
		for (set<Spot*>::const_iterator spot = spots.begin(); spot != spots.end(); spot++)
			(*spot)->update(elapsedTime);
		for (map<int, Box*>::iterator box = boxes.begin(); box != boxes.end(); box++)
			(box->second)->update(elapsedTime);
		
		testCollisions();

		// update beats
		updateBeat(elapsedTime);

	}

	void Pown::objectDetected(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			floor = object->getPolygons()[0];
			brickManager = new BrickManager(floor);
		}
		else
		{
			if (boxes.find(object->getId()) == boxes.end())
			{
				Box* box = new Box(object, NoteBeat(-1, -1));
				brickManager->updateBox(box);
				boxes[object->getId()] = box;
			}
		}
	}
		
	void Pown::objectUpdated(const IObjectPtr& object)
	{
		if (object->getId() != TABLE_ID)
		{
			map<int, Box*>::iterator b = boxes.find(object->getId());
			if (b != boxes.end())
				b->second->updateModelObject(object);
			brickManager->updateBox(b->second);
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
		map<int, Box*>::iterator b = boxes.find(object->getId());
		if (b != boxes.end())
			b->second->objectTouched(object, touchPoint);
	}

	void Pown::buttonPressed(const IButtonPtr& btn)
	{
	}
	void Pown::buttonReleased(const IButtonPtr& btn)
	{
	}

	void Pown::keyPressed(int key)
	{
	}
}
