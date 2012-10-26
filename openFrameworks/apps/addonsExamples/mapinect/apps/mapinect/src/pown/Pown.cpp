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

	static vector<int> palette;
	static vector<ofImage*> textures;
	static vector<ofImage*> colors;

	Pown::Pown()
		: brickManager(NULL), light(ambient, diffuse, ofVec3f(lightRadius, lightHeight, 0)), paused(false)
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
		palette.push_back(0xFF0000);	// red
		palette.push_back(0x00FF00);	// green
		palette.push_back(0x00FFFF);	// cyan
		palette.push_back(0xFFF000);	// orange
		palette.push_back(0xA448FF);	// violet
		palette.push_back(0xFFFF00);	// yellow
		palette.push_back(0xFF00FF);	// magenta
		palette.push_back(0x4AA0FF);	// lake blue

		for (int i = 1; i <= 8; i++)
		{
			ofImage* letra_i = new ofImage("data/textures/letra_" + ofToString(i) + ".png");
			textures.push_back(letra_i);
		}
		for (int i = 1; i <= 8; i++)
		{
			ofImage* color_i = new ofImage("data/textures/color_" + ofToString(i) + ".png");
			colors.push_back(color_i);
		}
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
		if (!paused)
		{
			// update all existing objects
			if (brickManager != NULL)
				brickManager->update(elapsedTime);
			for (map<int, Box*>::iterator box = boxes.begin(); box != boxes.end(); box++)
				(box->second)->update(elapsedTime);
		
			// update beats
			updateBeat(elapsedTime);
		}
	}

	bool sortBoxOnZ(Box* b1, Box* b2)
	{
		return b1->getCenter().z > b2->getCenter().z;
	}

	void Pown::orderBoxesFromLeftToRight()
	{
		vector<Box*> vBoxes;
		map<int,Box*>::iterator it;
		for (it = boxes.begin(); it != boxes.end(); it++)
		{
			vBoxes.push_back(it->second);
		}
		sort(vBoxes.begin(), vBoxes.end(),sortBoxOnZ);
		cout << "boxes size:" << vBoxes.size() << endl;
		for (int i = 0; i < vBoxes.size(); i++)
			if (i < textures.size()) {
				vBoxes[i]->setLetterTexture(textures[i]);
				vBoxes[i]->setColorTexture(colors[i]);				
			}
	}

	void Pown::objectDetected(const IObjectPtr& object)
	{
		if (!paused)
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
					ofFloatColor color = ofRandomColor();
					if (!palette.empty())
					{
						int c = *palette.rbegin();
						palette.pop_back();
						color.fromHex(c);
					}
					Box* box = new Box(object, color, NoteBeat(-1, -1), SoundManager::getProgram());
					brickManager->updateBox(box);
					boxes[object->getId()] = box;

					// Al agregar un nuevo objeto, reordenar
					orderBoxesFromLeftToRight();
				}
			}
		}
	}
		
	void Pown::objectUpdated(const IObjectPtr& object)
	{
		if (!paused)
		{
			if (object->getId() != TABLE_ID)
			{
				map<int, Box*>::iterator b = boxes.find(object->getId());
				if (b != boxes.end())
					b->second->updateModelObject(object);
				brickManager->updateBox(b->second);
				orderBoxesFromLeftToRight();
			}
		}
	}

	void Pown::objectLost(const IObjectPtr& object)
	{
		if (!paused)
		{
			if (object->getId() != TABLE_ID)
			{
				map<int, Box*>::iterator box = boxes.find(object->getId());
				if (box != boxes.end())
				{
					int c = box->second->getColor().getHex();
					palette.push_back(c);
					boxes.erase(object->getId());
					orderBoxesFromLeftToRight();
				}
			}
		}
	}

	void Pown::objectMoved(const IObjectPtr& object, const DataMovement& movement)
	{
	}
	
	void Pown::objectTouched(const IObjectPtr& object, const DataTouch& touchPoint)
	{
		if (!paused)
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
		if (key == ' ')
		{
			paused = !paused;
		}
	}
}
