#include "Box.h"

#include "ofGraphicsUtils.h"

namespace story
{
	Box::Box(const IObjectPtr& object, IButtonManager* btnManager)
		: object(object)
	{
		//texture = new ofImage("spot.png");
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
			ofImage* texture;
			IPolygonName face = (*p)->getName();
			switch(face)
			{
				case kPolygonNameTop:
					texture = textureTop;
					break;
				case kPolygonNameSideA:
					texture = textureA;
					break;
				case kPolygonNameSideB:
					texture = textureB;
					break;
				case kPolygonNameSideC:
					texture = textureC;
					break;
				case kPolygonNameSideD:
					texture = textureD;
					break;
				default:
					texture = textureA;
			}

			texture->bind();
			ofDrawQuadTextured((*p)->getMathModel().getVertexs(), ofTexCoordsFor(*texture));
			texture->unbind();
		}
	}

	void Box::update(float elapsedTime)
	{

	}

}
