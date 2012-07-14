#include "Box.h"

#include "ofGraphicsUtils.h"

#define BOLT_BOOST_COLOR_DECREASE_FACTOR	1.0f

namespace pown
{
	Box::Box(const IObjectPtr& object)
		: object(object)
	{
	}

	Box::~Box()
	{
		object.reset();
	}

	void Box::draw()
	{
		ofSetColor(color + boostColor);
		for (vector<IPolygonPtr>::const_iterator p = object->getPolygons().begin(); p != object->getPolygons().end(); ++p)
		{
			ofDrawQuad((*p)->getMathModel().getVertexs());
		}
	}

	void Box::update(float elapsedTime)
	{
		if (boostColor.getBrightness() > 0)
		{
			boostColor -= BOLT_BOOST_COLOR_DECREASE_FACTOR * elapsedTime;
		}
	}

	bool Box::testHit(Bolt* bolt)
	{
		IPolygonPtr bottomPolygon = object->getPolygon(kPolygonNameBottom);
		return bottomPolygon->getMathModel().isInPolygon(bottomPolygon->getMathModel().getPlane().project(bolt->getPosition()));
	}

	void Box::absorbBolt(Bolt* bolt)
	{
		boostColor += bolt->getColor() * bolt->getIntensity();
		bolt->absorb();
	}

}
