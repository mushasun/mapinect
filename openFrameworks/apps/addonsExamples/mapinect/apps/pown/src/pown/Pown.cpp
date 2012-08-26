#include "Pown.h"

#include "ofGraphicsUtils.h"
#include "ofLight.h"
#include "PownConstants.h"
#include "SoundManager.h"
#include "Timer.h"

namespace pown
{
	const float lightRadius = 0.25f;
	const float lightHeight = -0.4f;
	const ofFloatColor ambient(0.1f, 0.1f, 0.1f, 1.0f);
	const ofFloatColor diffuse(0.5f, 0.5f, 0.5f, 1.0f);

	Pown::Pown()
		: brickManager(NULL), light(ambient, diffuse, ofVec3f(lightRadius, lightHeight, 0))
	{
		floor.reset();
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
	}

	void Pown::draw()
	{
		ofEnableAlphaBlending();

		static ofLight debugLight;
		debugLight.setPosition(light.getPos());
		//debugLight.draw();

		if (brickManager != NULL)
			brickManager->draw(light);
		for (map<int, Box*>::iterator box = boxes.begin(); box != boxes.end(); box++)
			(box->second)->draw(light);
		
		ofDisableAlphaBlending();
	}

	void Pown::updateBeat(float elapsedTime)
	{
		if (brickManager != NULL)
		{
			static float beatTimer = 0;
			beatTimer += elapsedTime;

			static float emitTimer = 0;
			emitTimer += elapsedTime;
			float phi = 2.0f * PI * (emitTimer / (4.0f * PownConstants::EMIT_TIME));
			float xx = sin(phi) * lightRadius;
			float yy = lightHeight;
			float zz = cos(phi) * lightRadius;
			light.setPos(ofVec3f(xx, yy, zz));

			if (beatTimer >= PownConstants::BEAT_TIME)
			{
				beatTimer -= PownConstants::BEAT_TIME;

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
		for (map<int, Box*>::iterator box = boxes.begin(); box != boxes.end(); box++)
			(box->second)->update(elapsedTime);
		
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
				Box* box = new Box(object, NoteBeat(-1, -1), SoundManager::getProgram());
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
		if (object->getId() != TABLE_ID)
		{
			map<int, Box*>::iterator b = boxes.find(object->getId());
			if (b != boxes.end())
				b->second->objectTouched(object, touchPoint);
			if (touchPoint.getType() == kTouchTypeStarted)
				SoundManager::setProgram(ofRandom(PROGRAMS));
		}
	}

	void Pown::buttonPressed(const IButtonPtr& btn, const DataTouch& touchPoint)
	{
	}

	void Pown::buttonReleased(const IButtonPtr& btn, const DataTouch& touchPoint)
	{
	}

	void Pown::pointTouched(const DataTouch& touch)
	{
	}

	void Pown::keyPressed(int key)
	{
	}
}
