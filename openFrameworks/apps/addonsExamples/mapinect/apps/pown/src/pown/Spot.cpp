#include "Spot.h"

#include "ofGraphicsUtils.h"
#include "PownConstants.h"

namespace pown
{
	ofImage* Spot::texture = NULL;

	Spot::Spot(const ofVec3f& position)
		: position(position), box(NULL), rotation(0.0f)
	{
		const float size = 0.05f;
		vector<ofVec3f> vertexs;
		vertexs.push_back(ofVec3f(-size * 0.5f, 0, -size * 0.5f));
		vertexs.push_back(ofVec3f(size * 0.5f, 0, -size * 0.5f));
		vertexs.push_back(ofVec3f(size * 0.5f, 0, size * 0.5f));
		vertexs.push_back(ofVec3f(-size * 0.5f, 0, size * 0.5f));
		area = Polygon3D(vertexs);
	}

	Spot::~Spot()
	{
	}

	void Spot::setup()
	{
		texture = new ofImage("spot.png");
	}

	void Spot::draw()
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

	void Spot::update(float elapsedTime)
	{
		rotation += TWO_PI * elapsedTime / PownConstants::SPOT_PERIOD_TIME;
		if (rotation >= TWO_PI)
		{
			rotation -= TWO_PI;
		}
	}

	bool Spot::testHit(Box* box)
	{
		ofVec3f boxProjectedCenter(area.getPlane().project(box->getCenter()));
		return position.distance(boxProjectedCenter) < 0.05f;
	}

	void Spot::setBox(Box* box)
	{
		this->box = box;
	}

}
