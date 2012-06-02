#include "Floor.h"

#include "ofGraphicsUtils.h"

namespace buildings {

	GLuint Floor::floorTexture = 0;

	//--------------------------------------------------------------
	void Floor::draw(const ITxManager* txManager)
	{
		txManager->enableTextures();
		txManager->bindTexture(Floor::floorTexture);

		ofDrawQuadTextured(
			modelObject->getMathModel().getVertexs()[0],
			modelObject->getMathModel().getVertexs()[1],
			modelObject->getMathModel().getVertexs()[2],
			modelObject->getMathModel().getVertexs()[3]);

		txManager->disableTextures();
	}

}
