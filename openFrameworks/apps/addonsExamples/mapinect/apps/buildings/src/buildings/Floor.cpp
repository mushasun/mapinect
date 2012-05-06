#include "Floor.h"

#include "Table.h"

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

			ofDrawQuadTextured(q->getVertex(0), q->getVertex(1), q->getVertex(2), q->getVertex(3));

			txManager->disableTextures();
		}			
	}

}
