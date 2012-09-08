#include "Floor.h"

#include "ofGraphicsUtils.h"

namespace visualizer {

	ofImage* Floor::floorTexture = NULL;

	Floor::Floor(const IPolygonPtr& modelObject)
		: modelObject(modelObject)
	{
		if (floorTexture == NULL)
		{
			floorTexture = new ofImage("data/texturas/floor.jpg");
		}
	}

	//--------------------------------------------------------------
	void Floor::draw()
	{
		floorTexture->bind();

		ofDrawQuadTextured(modelObject->getMathModel().getVertexs(), ofTexCoordsFor());

		floorTexture->unbind();
	}

}
