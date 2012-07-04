#include "SimpleButton.h"
#include "pointutils.h"
#include "ofGraphicsUtils.h"

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
	void SimpleButton::init()
	{
		id = btnIds;
		btnIds++;
		leaderTouch = -1;
		leaderChanged = false;
	}

	SimpleButton::SimpleButton(Polygon3D polygon, ofColor idle, ofColor pressed):
	idleColor(idle), currentColor(idle), pressedColor(pressed), polygon(polygon)
	{
		init();
		mode = kColor;
	}

	SimpleButton::SimpleButton(Polygon3D polygon, ofImage* idle, ofImage* pressed):
	texPressed(pressed), texIdle(idle), polygon(polygon)
	{
		init();
		mode = kBgImg;
	}

		
	ButtonEvent SimpleButton::updateTouchPoints(DataTouch touch)
	{
		int newLeader = leaderTouch;
		ButtonEvent evnt = NO_CHANGE;
		if(polygon.isInPolygon(touch.getTouchPoint()))
		{
			if(touch.getType() == kTouchTypeReleased &&
				contacts.size() > 0)
			{
				contacts.erase(touch.getId());
				if(leaderTouch == touch.getId())
				{
					if(contacts.size() == 0)
						newLeader = -1;
					else
						newLeader = contacts.begin()->second.getId();

					//cout << "erease: " << touch.getId() << " - " << newLeader << endl;
				}
			}
			else
			{
				contacts[touch.getId()] = touch;
				if(leaderTouch == -1)
					newLeader = touch.getId();

				//cout << "added: " << touch.getId() << " - " << newLeader << endl;

			}
				//contacts.insert(pair<int,DataTouch>(touch.getId(),touch));

			if(contacts.size() == 0)
				evnt = RELEASED;
			else if(contacts.size() == 1 &&
					touch.getType() == kTouchTypeStarted)
				evnt = PRESSED;
			else
				evnt = NO_CHANGE;
		}
		else
		{
			map<int,DataTouch>::iterator contact = contacts.find(touch.getId());
			if(contact != contacts.end())
			{
				contacts.erase(touch.getId());
				if(leaderTouch == touch.getId())
				{
					if(contacts.size() == 0)
						newLeader = -1;
					else
						newLeader = contacts.begin()->second.getId();
				}

				//cout << "erease: " << touch.getId() << " - " << newLeader << endl;

				if(contacts.size() == 0)
					evnt = RELEASED;
			}
		}

		if(newLeader != leaderTouch)
		{
			leaderTouch = newLeader;
			leaderChanged = true;
			cout << "Leader changed : " << leaderChanged << "- " << leaderTouch << endl;
		}
		else
			leaderChanged = false;

		
		cout << "Leader: " << leaderTouch << endl;
		return evnt;
	}

	void SimpleButton::draw()
	{
		vector<ofVec3f> vertexs = polygon.getVertexs();

		switch(mode)
		{
			case kColor:
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
				break;
			case kBgImg:
				ofSetColor(255,255,255);
				ofImage* tex = isPressed() ? texPressed : texIdle;
				tex->bind();
				vector<ofVec2f> texCoords(ofTexCoordsFor(*tex));
				ofDrawQuadTextured(vertexs, texCoords);
				tex->unbind();
				break;
		}
		
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