#include "Floor.h"

#include "ofGraphicsUtils.h"

namespace buildings {

	ofImage* Floor::floorTexture = NULL;

	Floor::Floor(const IPolygonPtr& modelObject)
		: modelObject(modelObject)
	{
		if (floorTexture == NULL)
		{
			floorTexture = new ofImage("data/texturas/135367.jpg");
		}
	}

	//--------------------------------------------------------------
	void Floor::draw(const ITxManager* txManager)
	{
		floorTexture->bind();

		ofDrawQuadTextured(modelObject->getMathModel().getVertexs(), ofTexCoordsFor(*floorTexture));

		floorTexture->unbind();
	}

}
