#include "Box.h"

#include "ofGraphicsUtils.h"
#include "PownConstants.h"
#include "SoundManager.h"

namespace pown
{
	const ofFloatColor kBaseBoostColor = kRGBMidGray;

	Box::Box(const IObjectPtr& object, const ofFloatColor& color, const NoteBeat& noteBeat, int program)
		: object(object), color(color), noteBeat(noteBeat), program(program), boostColor(0), letterTexture(NULL), colorTexture(NULL)
	{
	}

	Box::~Box()
	{
		object.reset();
	}

	void Box::draw(const Light& light) const
	{
		//ofDrawQuad(object->getPolygon(kPolygonNameBottom)->getMathModel().getVertexs());
		for (vector<IPolygonPtr>::const_iterator p = object->getPolygons().begin(); p != object->getPolygons().end(); ++p)
		{
			IPolygonPtr polygon = *p;
			const vector<ofVec3f>& vxs(polygon->getMathModel().getVertexs());
			
			ofSetColor(kRGBWhite);
			if (polygon->getName() == kPolygonNameSideA && letterTexture != NULL)
			{
				letterTexture->bind();
				ofDrawQuadTextured(vxs, ofTexCoordsFor());
				letterTexture->unbind();
			} else {
				colorTexture->bind();
				ofDrawQuadTextured(vxs, ofTexCoordsFor());
				colorTexture->unbind();

/*				ofSetColor(light.getColor(color,
					polygon->getMathModel().getCentroid(),
					polygon->getMathModel().getPlane().getNormal()) + boostColor);
				ofDrawQuad(vxs);
*/
			}
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

	void Box::objectTouched(const IObjectPtr& object, const DataTouch& touchPoint)
	{
		if (touchPoint.getType() == kTouchTypeStarted)
		{
			color = ofRandomColor();
		}
	}

	void Box::doBeat()
	{
		boostColor = kBaseBoostColor;
		SoundManager::playNote(noteBeat.note, program);
	}

	bool Box::testHit(Brick* brick) const
	{
		ofVec3f projected(brick->getHitPolygon().getPlane().project(getCenter()));
		bool result = brick->getHitPolygon().isInPolygon(projected);
		return result;
	}

}
