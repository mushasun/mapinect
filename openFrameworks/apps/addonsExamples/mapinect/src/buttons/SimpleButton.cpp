#include "SimpleButton.h"
#include "pointutils.h"
#include "ofGraphicsUtils.h"

namespace mapinect {

	void SimpleButton::init()
	{

	}

	SimpleButton::SimpleButton(Polygon3D polygon, ofColor idle, ofColor pressed):
	BaseButton(idle,pressed), polygon(polygon)
	{
		init();
		
	}

	SimpleButton::SimpleButton(Polygon3D polygon, ofImage* idle, ofImage* pressed):
	BaseButton(idle,pressed), polygon(polygon)
	{
		init();
	}

	ButtonEvent SimpleButton::updateTouchPoints(const DataTouch& touch)
	{
		return BaseButton::updateTouchPoints(touch);
	}

	bool SimpleButton::isInTouch(const DataTouch& touch)
	{
		return polygon.isInPolygon(touch.getTouchPoint());
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


}