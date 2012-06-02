#include "Drawing.h"

#include "Globals.h"

namespace drawing {
	
	//--------------------------------------------------------------
	Drawing::Drawing()
	{
	}

	//--------------------------------------------------------------
	Drawing::~Drawing()
	{
	}

	//--------------------------------------------------------------
	void Drawing::setup()
	{
		for (int i = 0; i < 6; i++)
		{
			ofSoundPlayer sound;
			sound.loadSound("sounds/sound" + ofToString(i) + ".mp3");
			sounds.push_back(sound);
		}
	}

	//--------------------------------------------------------------
	void Drawing::exit()
	{
	}

	//--------------------------------------------------------------
	void Drawing::debugDraw()
	{
		map<int, DataTouch> keep;
		for (map<int, DataTouch>::const_iterator it = touchPoints.begin(); it != touchPoints.end(); ++it)
		{
			if (it->second.getType() == kTouchTypeStarted)
				ofSetHexColor(0xFF0000);
			else if (it->second.getType() == kTouchTypeHolding)
				ofSetHexColor(0x00FF00);
			else
				ofSetHexColor(0x0000FF);
			ofVec2f s = gKinect->getScreenCoordsFromWorldCoords(it->second.getTouchPoint());
			ofCircle(s.x, s.y, 4);
			if (it->second.getType() != kTouchTypeReleased)
				keep.insert(make_pair(it->first, it->second));
		}
		touchPoints = keep;
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
		map<int, DataTouch>::iterator it = touchPoints.find(touchPoint.getId());
		if (it == touchPoints.end())
		{
			assert(touchPoint.getType() == kTouchTypeStarted);
			touchPoints.insert(make_pair(touchPoint.getId(), touchPoint));
			static int s = 0;
			sounds[s].play();
			s = (s + 1) % sounds.size();
		}
		else
		{
			it->second = touchPoint;
		}
	}
}
