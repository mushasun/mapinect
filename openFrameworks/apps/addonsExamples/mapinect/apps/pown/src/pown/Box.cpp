#include "Box.h"

#include "ofGraphicsUtils.h"
#include "PownConstants.h"

#include "utils.h"

namespace pown
{
	Box::Box(const IObjectPtr& object, const ofColor& color)
		: object(object), color(color), boostColor(0)
	{
	}

	Box::~Box()
	{
		object.reset();
	}

	void Box::draw()
	{
		ofSetColor(color + boostColor);
		//ofDrawQuad(object->getPolygon(kPolygonNameBottom)->getMathModel().getVertexs());
		for (vector<IPolygonPtr>::const_iterator p = object->getPolygons().begin(); p != object->getPolygons().end(); ++p)
		{
			ofDrawQuad((*p)->getMathModel().getVertexs());
		}
	}

	void Box::update(float elapsedTime)
	{
		if (boostColor.getBrightness() > 0)
		{
			float r = boostColor.r - PownConstants::BOLT_BOOST_COLOR_DECREASE_FACTOR * elapsedTime;
			float g = boostColor.g - PownConstants::BOLT_BOOST_COLOR_DECREASE_FACTOR * elapsedTime;
			float b = boostColor.b - PownConstants::BOLT_BOOST_COLOR_DECREASE_FACTOR * elapsedTime;
			boostColor.r = max<float>(r, 0);
			boostColor.g = max<float>(g, 0);
			boostColor.b = max<float>(b, 0);
		}
	}

	bool Box::testHit(Bolt* bolt)
	{
		IPolygonPtr bottomPolygon = object->getPolygon(kPolygonNameBottom);
		ofVec3f projected(bottomPolygon->getMathModel().getPlane().project(bolt->getPosition()));
		bool result = bottomPolygon->getMathModel().isInPolygon(projected);
		return result;
	}

	void Box::absorbBolt(Bolt* bolt)
	{
		ofColor boost = bolt->getColor() * bolt->getIntensity();
		boostColor.r = min(255, boostColor.r + boost.r + color.r) - color.r;
		boostColor.g = min(255, boostColor.g + boost.g + color.g) - color.g;
		boostColor.b = min(255, boostColor.b + boost.b + color.b) - color.b;
		bolt->absorb();
	}

}
