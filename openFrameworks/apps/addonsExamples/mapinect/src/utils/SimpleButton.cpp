#include "SimpleButton.h"
#include "pointutils.h"

namespace mapinect {
	//
	//SimpleButton::SimpleButton(ofVec3f pos, float height, float width, Plane3D plane, ofColor idle, ofColor pressed):
	//	idleColor(idle), currentColor(idle), pressedColor(pressed), width(width), height(height), plane(plane), position(pos)
	//{
	//	calculatePolygon();

	//	id = btnIds;
	//	btnIds++;
	//	isPressed = false;
	//	isTouching = false;
	//}

	SimpleButton::SimpleButton(Polygon3D polygon, ofColor idle, ofColor pressed):
	idleColor(idle), currentColor(idle), pressedColor(pressed), polygon(polygon)
	{
		id = btnIds;
		btnIds++;
		//contacts = 0;
	}
		
	ButtonEvent SimpleButton::updateTouchPoints(DataTouch touch)
	{
		if(polygon.isInPolygon(touch.getTouchPoint()))
		{
			if(touch.getType() == kTouchTypeReleased &&
				contacts.size() > 0)
				contacts.erase(touch.getId());
			else
				contacts[touch.getId()] = touch;
				//contacts.insert(pair<int,DataTouch>(touch.getId(),touch));

			if(contacts.size() == 0)
				return RELEASED;
			else if(contacts.size() == 1 &&
					touch.getType() == kTouchTypeStarted)
				return PRESSED;
			else
				return NO_CHANGE;
		}
		else
		{
			map<int,DataTouch>::iterator contact = contacts.find(touch.getId());
			if(contact != contacts.end())
			{
				contacts.erase(contact->first);
				if(contacts.size() == 0)
					return RELEASED;
			}
		}

		return NO_CHANGE;
	}

	void SimpleButton::draw()
	{
		vector<ofVec3f> vertexs = polygon.getVertexs();
		if(isPressed())
			ofSetColor(pressedColor);
		else
			ofSetColor(idleColor);

		glBegin(GL_QUADS);      
			glVertex3f(vertexs.at(0).x, vertexs.at(0).y, vertexs.at(0).z); 
			glVertex3f(vertexs.at(1).x, vertexs.at(1).y, vertexs.at(1).z);
			glVertex3f(vertexs.at(2).x, vertexs.at(2).y, vertexs.at(2).z);
			glVertex3f(vertexs.at(3).x, vertexs.at(3).y, vertexs.at(3).z);
		glEnd();
	}

	//void SimpleButton::setPosition(ofVec3f pos)
	//{
	//	position = pos;
	//	calculatePolygon();
	//}

	//void SimpleButton::calculatePolygon()
	//{
	//	vector<ofVec3f> vertexs;
	//	vertexs.push_back(position);
	//	vertexs.push_back(plane.project(position + ofVec3f(position.x + width,0,0)));
	//	vertexs.push_back(plane.project(position + ofVec3f(position.x + width,0,position.z + height)));
	//	vertexs.push_back(plane.project(position + ofVec3f(0,0,position.z + height)));

	//	polygon = Polygon3D(vertexs);
	//}
}