#include "tejo.h"

Tejo::Tejo(ofxVec3f position, double radio, ofxVec3f direction, double vel, ofxVec3f tableNorm, ofxVec3f tableCenter)
{
	this->position = position;
	this->radio = radio;
	this->direction = direction.normalize();
	this->vel = vel;
	this->tableNormal = tableNorm;
	this->tableCenter = tableCenter;
}

void Tejo::draw()
{
	glPushMatrix();
	ofTranslate(position.x*1000,-position.y*1000,-position.z*1000);
	ofSetColor(0,255,0);
	glutSolidSphere(radio*1000,100,100);
	ofPopMatrix();
}


void Tejo::update(vector<BObject*> bobjects)
{
	if(check_collision(bobjects))
		cout << "Choco!" << endl;
	ofxVec3f dif = position - tableCenter;
	ofxVec3f proj = dif.dot(tableNormal) * tableNormal;
	position = position - proj;
	move();
}

void Tejo::move()
{

	position += direction * vel;
}

bool Tejo::check_collision(vector<BObject*> bobjects){ 
	ofxVec3f normal;
	bool collision = false;
	for(int j = 0; j < bobjects.size(); j++)
	{
		BObject* curObj = bobjects.at(j);
		vector<Segment3D> segmentos = curObj->getSegments();
		for(int i = 0; i < segmentos.size(); i++)
		{
			Segment3D s = segmentos.at(i);

			/*ofxVec3f pt_v = (position + direction*vel) - s.getOrigin();
			ofxVec3f proj_v = (pt_v.dot(s.getDirection())) * s.getDirection(); 
			ofxVec3f closest = s.getOrigin() + proj_v;
			float dist = (position - closest).length();*/

			ofxVec3f closest = s.closestPoint(position+direction*vel);
			float dist = (position+direction*vel - closest).length();
			if(dist <= radio)
			{
				if(direction.dot(s.getNormal()) < 0)
				{
					normal += s.getNormal();
					collision = true;
					curObj->sound.play();
					curObj->setColorBoost(60);
				}
				else if(s.isDoubleNormal())
				{
					normal -= s.getNormal();
					collision = true;
					curObj->setColorBoost(60);
					curObj->sound.play();
				}
			}
		}
	}
	if(collision)
	{
		normal.normalize();
		direction = (-2 * direction.dot(normal) * normal) + direction;
		direction.normalize();
	}
	return collision;
}