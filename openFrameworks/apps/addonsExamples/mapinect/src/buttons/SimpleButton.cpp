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

	ButtonEvent SimpleButton::updateTouchPoints(const IObjectPtr& object, const DataTouch& touch)
	{
		return BaseButton::updateTouchPoints(object, touch);
	}

	bool SimpleButton::isInTouch(const IObjectPtr& object, const DataTouch& touch)
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

				ofDrawQuad(vertexs);
				break;
			case kButtonDrawModeTextured:
				ofSetColor(kRGBWhite);
				ofImage* tex = isPressed() ? pressedTexture: idleTexture;
				tex->bind();
				ofDrawQuadTextured(vertexs, texCoords);
				tex->unbind();
				break;
		}
		
	}


}