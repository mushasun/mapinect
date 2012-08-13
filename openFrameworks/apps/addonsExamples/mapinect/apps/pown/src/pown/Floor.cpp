#include "Floor.h"

#include "ofGraphicsUtils.h"
#include "PownConstants.h"
#include "SoundManager.h"

namespace pown
{
	Floor::Floor(const IObjectPtr& object, const ofColor& color)
		object(object->getPolygons()[0]), baseColor(color)
	{

	}

	Floor::~Floor()
	{
	}

	void Floor::update(float elapsedTime)
	{
		for (vector<FloorBrick>::iterator fb = bricks.begin(); fb != bricks.end(); fb++)
			fb->update(elapsedTime);
	}

	void Floor::draw() const
	{
		for (vector<FloorBrick>::const_iterator fb = bricks.begin(); fb != bricks.end(); fb++)
			fb->draw();
	}

	bool Floor::testHit(Bolt* bolt) const
	{
		ofVec3f projected(polygon->getMathModel().getPlane().project(bolt->getPosition()));
		bool result = !polygon->getMathModel().isInPolygon(projected);
		return result;
	}

	FloorBrick::FloorBrick(const Polygon3D& polygon, const ofColor& color)
		: polygon(polygon), color(color)
	{
	}

	void FloorBrick::update(float elapsedTime)
	{
	}

	void FloorBrick::draw() const
	{
		ofSetColor(color);
		ofDrawQuad(polygon.getVertexs());
	}

}
