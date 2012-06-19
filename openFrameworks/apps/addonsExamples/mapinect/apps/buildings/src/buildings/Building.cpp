#include "Building.h"

#include "ofGraphicsUtils.h"
#include "ofVecUtils.h"

namespace buildings {

	ofImage* Building::buildingTexture = NULL;
	ofImage* Building::roofTexture = NULL;

	Building::Building(const IObjectPtr& object)
		: object(object), progress(0)
	{
		if (buildingTexture == NULL)
		{
			buildingTexture = new ofImage("data/texturas/Building_texture.jpg");
		}
		if (roofTexture == NULL)
		{
			roofTexture = new ofImage("data/texturas/oba.jpg");
		}
	}

	Building::~Building() {

	}

	void Building::draw(const ITxManager* txManager, const Floor& floor)
	{
		if (progress < 1.0f) {
			progress += 0.002f;
		}

		for (vector<IPolygonPtr>::const_iterator p = object->getPolygons().begin(); p != object->getPolygons().end(); ++p)
		{
			static ofVec3f vA,vB,vC,vD;

			vA = (*p)->getMathModel().getVertexs()[0];
			vB = (*p)->getMathModel().getVertexs()[1];
			vC = (*p)->getMathModel().getVertexs()[2];
			vD = (*p)->getMathModel().getVertexs()[3];

			ofVec3f floorNormal = floor.getModelObject()->getMathModel().getPlane().getNormal();
			float prod = abs((*p)->getMathModel().getPlane().getNormal().dot(floorNormal));
			//prod = 1.0;
			if (prod < 0.9)
			{
				/*	__________
					|2		3|
					|		 |
					|		 |
					|1		0|
				*/

				ofVec3f v0 = vB;
				ofVec3f v1 = vC;
				ofVec3f v2 = vD;
				ofVec3f v3 = vA;

				// Bind Texture
				buildingTexture->bind();

				ofVec3f h12 = v2 - v1;
				ofVec3f h03 = v3 - v0;			
			
				ofVec3f v3p = v3 - (h03*(1-progress)); //(v3.x, v0.y - progress * h03, v3.z);
				ofVec3f v2p = v2 - (h12*(1-progress)); //(v2.x, v1.y - progress * h12, v2.z);
				
				vector<ofVec2f> texCoords(ofTexCoordsFor(*buildingTexture));
				for (vector<ofVec2f>::iterator t = texCoords.begin(); t != texCoords.end(); ++t)
				{
					*t *= progress;
				}

				vector<ofVec3f> vertexs;
				vertexs.push_back(v1);
				vertexs.push_back(v0);
				vertexs.push_back(v3p);
				vertexs.push_back(v2p);

				ofDrawQuadTextured(vertexs, texCoords);

				buildingTexture->unbind();
			}
			else
			{
				roofTexture->bind();
				ofDrawQuadTextured((*p)->getMathModel().getVertexs(), ofTexCoordsFor(*roofTexture));
				roofTexture->unbind();
			}
		}
	}

	void Building::update() {
	
	}

}
