#include "BObject.h"
#include "utils.h"

BObject::BObject(std::vector<Segment3D> segments, ofxVec3f color, int id, int soundId)
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
	gModel->objectsMutex.lock();
	if(polyhedron != NULL)
	{
		for (int i = 0; i < this->polyhedron->getPCPolygonSize(); i++)
		{
			PCPolygon* gon = polyhedron->getPCPolygon(i);
			if (gon->hasObject()) 
			{
				mapinect::Polygon* q = gon->getPolygonModelObject();
				ofxVec3f vA = q->getVertex(0)*ofxVec3f(1,-1,-1)*1000;
				ofxVec3f vB = q->getVertex(1)*ofxVec3f(1,-1,-1)*1000;
				ofxVec3f vC = q->getVertex(2)*ofxVec3f(1,-1,-1)*1000;
				ofxVec3f vD = q->getVertex(3)*ofxVec3f(1,-1,-1)*1000;
				ofxVec3f boostedColor = color + colorBoost;
				ofSetColor(boostedColor.x,boostedColor.y,boostedColor.z);
				glBegin(GL_TRIANGLES);      
					glVertex3f(vA.x,vA.y,vA.z);    
					glVertex3f(vB.x,vB.y,vB.z);
					glVertex3f(vC.x,vC.y,vC.z);
					glVertex3f(vC.x,vC.y,vC.z);
					glVertex3f(vD.x,vD.y,vD.z);
					glVertex3f(vA.x,vA.y,vA.z);
				glEnd();
			}			
		}
	}
	gModel->objectsMutex.unlock();
}




