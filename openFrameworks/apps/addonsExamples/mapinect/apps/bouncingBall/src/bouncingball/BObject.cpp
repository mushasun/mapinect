#include "BObject.h"

namespace bouncing {
	BObject::BObject(std::vector<Segment3D> segments, ofVec3f color, int id, int soundId)
	{
		this->segments = segments;
		this->color = color;
		this->id = id;
		this->visited = true;
		sound.loadSound("sounds/sound" + ofToString(soundId) + ".mp3");
		this->polyhedron = NULL;
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
		if(polyhedron != NULL)
		{
			for (int i = 0; i < this->polyhedron->getPCPolygonSize(); i++)
			{
				PCPolygon* gon = polyhedron->getPCPolygon(i);
				if (gon->hasObject()) 
				{
					mapinect::Polygon* q = gon->getPolygonModelObject();
					ofVec3f boostedColor = color + colorBoost;
					ofSetColor(boostedColor.x,boostedColor.y,boostedColor.z);
					ofDrawQuadTextured(q->getVertex(0), q->getVertex(1), q->getVertex(2), q->getVertex(3));
				}			
			}
		}
	}

}
