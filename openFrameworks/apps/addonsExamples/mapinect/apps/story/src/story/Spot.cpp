#include "Spot.h"

#include "ofGraphicsUtils.h"
#include "StoryConstants.h"

namespace story
{
	ofImage* Spot::texture = NULL;

	Spot::Spot() : rotation(0.0f), size(0.05), active(false)
	{
	}

	Spot::~Spot()
	{
	}

	void Spot::setup()
	{
		texture = new ofImage("data/texturas/spot.png");
	}

	void Spot::draw()
	{
		if(active)
		{
			ofSetColor(kRGBWhite);

			ofPushMatrix();
				ofTranslate(position);
				ofRotateY(RAD2DEG(rotation));
				texture->bind();
					ofDrawQuadTextured(area.getVertexs(), ofTexCoordsFor(*texture));
				texture->unbind();
			ofPopMatrix();
		}

	}

	void Spot::update(float elapsedTime)
	{
		rotation += TWO_PI * elapsedTime / StoryConstants::SPOT_ROTATION_PERIOD_TIME;
		if (rotation >= TWO_PI)
		{
			rotation -= TWO_PI;
		}
	}

	void Spot::setPosition(const ofVec3f& pos) 
	{ 
		active = true;
		position = pos; 
		
		vector<ofVec3f> vertexs;
		vertexs.push_back(ofVec3f(-size * 0.5f, 0, -size * 0.5f));
		vertexs.push_back(ofVec3f(size * 0.5f, 0, -size * 0.5f));
		vertexs.push_back(ofVec3f(size * 0.5f, 0, size * 0.5f));
		vertexs.push_back(ofVec3f(-size * 0.5f, 0, size * 0.5f));
		area = Polygon3D(vertexs);
	}
}
