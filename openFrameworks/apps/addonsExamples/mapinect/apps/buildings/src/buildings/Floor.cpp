#include "Floor.h"

#include "Table.h"

#include "ofGraphicsUtils.h"

namespace buildings {

	GLuint Floor::floorTexture = 0;

	//--------------------------------------------------------------
	void Floor::draw(const ITxManager* txManager)
	{
		if (modelObject->hasObject()) 
		{
			mapinect::Polygon* q = modelObject->getPolygonModelObject();

			txManager->enableTextures();
			txManager->bindTexture(Floor::floorTexture);

			ofDrawQuadTextured(q->getVertexs()[0], q->getVertexs()[1], q->getVertexs()[2], q->getVertexs()[3]);

			txManager->disableTextures();
		}			
	}

}
