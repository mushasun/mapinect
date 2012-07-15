#include "Pown.h"

namespace pown
{
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
		for (set<Spot*>::const_iterator spot = spots.begin(); spot != spots.end(); spot++)
			(*spot)->draw();
		for (set<Bolt*>::const_iterator bolt = bolts.begin(); bolt != bolts.end(); bolt++)
			(*bolt)->draw();
		for (map<int, Box*>::iterator iter = boxes.begin(); iter != boxes.end(); iter++)
			(iter->second)->draw();
	}

	void Pown::update(float elapsedTime)
	{
	}

	void Pown::objectDetected(const IObjectPtr& object)
	{
		if (object->getId() != TABLE_ID)
		{
			if (boxes.find(object->getId()) == boxes.end())
			{
				boxes[object->getId()] = new Box(object);
			}
		}
	}
		
	//--------------------------------------------------------------
	void Pown::objectUpdated(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			if (floor != NULL)
			{
				floor->updateModelObject(object->getPolygons()[0]);
			}
		}
		else
		{
			map<int, Box*>::iterator b = boxes.find(object->getId());
			if (b != boxes.end())
				b->second->updateModelObject(object);
		}
	}

	//--------------------------------------------------------------
	void Pown::objectLost(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			if (floor != NULL)
			{
				delete floor;
				floor = NULL;
			}
		}
		else
		{
			if (boxes.find(object->getId()) != boxes.end())
			{
				boxes.erase(object->getId());
			}
		}
	}

	//--------------------------------------------------------------
	void Pown::objectMoved(const IObjectPtr& object, const DataMovement& movement)
	{
		//cout << "objMoved: " << movement.getTranslation().length() << endl;
		if (object->getId() == TABLE_ID)
		{
			if (floor != NULL)
			{
				floor->updateModelObject(object->getPolygons()[0]);
			}
		}
		else
		{
			map<int, Box*>::iterator b = boxes.find(object->getId());
			if (b != boxes.end())
			{
				b->second->updateModelObject(object);
			}
		}
	}
	
	//--------------------------------------------------------------
	void Pown::objectTouched(const IObjectPtr& object, const DataTouch& touchPoint)
	{
		map<int, DataTouch>::iterator it = touchPoints.find(touchPoint.getId());
		if (it == touchPoints.end())
		{
			//assert(touchPoint.getType() == kTouchTypeStarted);
			touchPoints.insert(make_pair(touchPoint.getId(), touchPoint));
		}
		else
		{
			it->second = touchPoint;
		}
	}

	void Pown::buttonPressed(const IButtonPtr& btn)
	{
		/*music.setSpeed(btn->getId());
		if(btn->getId() == 1 && !music.getIsPlaying())
			music.play();
		if(btn->getId() == 4)
			music.stop();

		int id = btn->getId();
		switch(id)
		{
			case 0: 
				if(sample1.getIsPlaying())
					sample1.stop();
				else
					sample1.play();
				break;
			case 1: 
				if(sample2.getIsPlaying())
					sample2.stop();
				else
					sample2.play();
				break;
			case 2: 
				if(sample3.getIsPlaying())
					sample3.stop();
				else
					sample3.play();
				break;
			case 3: 
				if(sample4.getIsPlaying())
					sample4.stop();
				else
					sample4.play();
				break;
		}*/
	}
	void Pown::buttonReleased(const IButtonPtr& btn)
	{
		cout << "release: " << btn->getId() << endl;
		switch(btn->getId())
		{
			case 2:
				if(tracks[currentTrack].getIsPlaying())
					tracks[currentTrack].stop();
				else
					tracks[currentTrack].play();
				break;
			case 3:
				tracks[currentTrack].stop();
				currentTrack--;
				currentTrack = currentTrack == -1 ? tracks.size() - 1 : currentTrack; 
				cout << "track: " << currentTrack;

				tracks[currentTrack].play();
				break;
			case 1:
				tracks[currentTrack].stop();
				currentTrack++;
				currentTrack %= tracks.size(); 

				cout << "track: " << currentTrack;
				tracks[currentTrack].play();
				break;

				
		}
	}
}
