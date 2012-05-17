#include "BObject.h"

#include "ofGraphicsUtils.h"

namespace bouncing
{
	BObject::BObject(const vector<Segment3D>& segments, const ofVec3f& color, int id, int soundId)
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
			for (int i = 0; i < modelObject->getPCPolygonSize(); i++)
			{
				mapinect::Polygon* poly = modelObject->getPCPolygon(i)->getPolygonModelObject();
				ofVec3f boostedColor = color + colorBoost;
				ofSetColor(boostedColor.x, boostedColor.y, boostedColor.z);
				ofDrawQuadTextured(
					poly->getVertexs()[0],
					poly->getVertexs()[1],
					poly->getVertexs()[2],
					poly->getVertexs()[3]);
			}
		}
		else if (modelObjectTable != NULL)
		{
			ofVec3f boostedColor = color + colorBoost;
			ofSetColor(boostedColor.x, boostedColor.y, boostedColor.z);
			mapinect::Polygon* poly = modelObjectTable->getPolygonModelObject();
			ofDrawQuadTextured(
				poly->getVertexs()[0],
				poly->getVertexs()[1],
				poly->getVertexs()[2],
				poly->getVertexs()[3]);
		}
	}

}
