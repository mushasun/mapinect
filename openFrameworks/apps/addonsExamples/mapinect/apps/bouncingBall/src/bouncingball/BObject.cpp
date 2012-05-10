#include "BObject.h"

namespace bouncing {
	BObject::BObject(const vector<Segment3D>& segments, ofVec3f color, int id, int soundId)
		: segments(segments), color(color), id(id)
	{
		visited = true;
		sound.loadSound("sounds/sound" + ofToString(soundId) + ".mp3");
		modelObject = NULL;
		lastTime = 0;
		colorBoost = 0;
	}

	void BObject::update()
	{
	
		if(colorBoost > 0)
		{
			if(lastTime > 0)
			{
				int elapsedTime = ofGetElapsedTimeMillis() - lastTime;
				colorBoost -= elapsedTime / 20;
				if(colorBoost < 0)
					colorBoost = 0;
			}
		
			lastTime = ofGetElapsedTimeMillis();
		}
		else 
			lastTime = 0;
	}

	void BObject::setColorBoost(int boost)
	{
		colorBoost += boost;
		if(colorBoost > 200)
			colorBoost = 200;
	}


	void BObject::draw()
	{
		if(modelObject != NULL)
		{
			ofVec3f boostedColor = color + colorBoost;
			ofSetHexColor(boostedColor.x, boostedColor.y, boostedColor.z);
			ofDrawQuadTextured(
				modelObject->getVertex(0),
				modelObject->getVertex(1),
				modelObject->getVertex(2),
				modelObject->getVertex(3));
		}
	}

}
