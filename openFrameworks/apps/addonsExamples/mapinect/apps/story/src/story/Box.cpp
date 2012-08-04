#include "Box.h"

#include "ofGraphicsUtils.h"

namespace story
{
	Box::Box(const IObjectPtr& object)
		: object(object)
	{
		texture = new ofImage("spot.png");
	}

	Box::~Box()
	{
		object.reset();
	}

	void Box::draw()
	{
		ofSetColor(kRGBWhite);

		for (vector<IPolygonPtr>::const_iterator p = object->getPolygons().begin(); p != object->getPolygons().end(); ++p)
		{
			texture->bind();
			ofDrawQuadTextured((*p)->getMathModel().getVertexs(), ofTexCoordsFor(*texture));
			texture->unbind();
		}
	}

	void Box::update(float elapsedTime)
	{
	}

}
