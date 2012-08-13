#include "SimpleButton.h"

#include "ofGraphicsUtils.h"

namespace mapinect {

	SimpleButton::SimpleButton()
	{
	}

	void SimpleButton::init()
	{
	}

	SimpleButton::SimpleButton(const Polygon3D& polygon, const ofColor& idle, const ofColor& pressed)
		: BaseButton(idle, pressed), polygon(polygon)
	{
		init();
	}

	SimpleButton::SimpleButton(const Polygon3D& polygon, ofImage* idle, ofImage* pressed)
		: BaseButton(idle, pressed), polygon(polygon)
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
			case kButtonDrawModePlain:
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
			case kButtonDrawModeTextured:
				ofSetColor(kRGBWhite);
				ofImage* tex = isPressed() ? pressedTexture: idleTexture;
				tex->bind();
				vector<ofVec2f> texCoords(ofTexCoordsFor(*tex));
				ofDrawQuadTextured(vertexs, texCoords);
				tex->unbind();
				break;
		}
		
	}


}