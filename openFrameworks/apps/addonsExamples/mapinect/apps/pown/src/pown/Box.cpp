#include "Box.h"

#include "ofGraphicsUtils.h"
#include "PownConstants.h"
#include "SoundManager.h"

namespace pown
{
	const ofFloatColor kBaseBoostColor = kRGBDarkGray;

	Box::Box(const IObjectPtr& object, const ofColor& color, const NoteBeat& noteBeat)
		: object(object), color(color), noteBeat(noteBeat), boostColor(0), program(0)
	{
	}

	Box::~Box()
	{
		object.reset();
	}

	void Box::draw() const
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
		if (boostColor.getBrightness() > 0.0f)
		{
			float r = boostColor.r - PownConstants::BOX_BEAT_TIME * elapsedTime;
			float g = boostColor.g - PownConstants::BOX_BEAT_TIME * elapsedTime;
			float b = boostColor.b - PownConstants::BOX_BEAT_TIME * elapsedTime;
			boostColor.r = max<float>(r, 0);
			boostColor.g = max<float>(g, 0);
			boostColor.b = max<float>(b, 0);
		}
	}

	void Box::doBeat()
	{
		boostColor = kBaseBoostColor;
		SoundManager::playNote(noteBeat.note, program);
	}

	bool Box::testHit(Brick* brick) const
	{
		ofVec3f projected(brick->getPolygon().getPlane().project(getCenter()));
		bool result = brick->getPolygon().isInPolygon(projected);
		return result;
	}

}
