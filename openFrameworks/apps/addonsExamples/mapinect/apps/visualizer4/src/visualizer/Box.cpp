#include "Box.h"

#include "ofGraphicsUtils.h"
#include "ofVecUtils.h"

namespace visualizer {

	ofImage* Box::barTexture = NULL;
	ofImage* Box::roofTexture = NULL;

	Box::Box(const IObjectPtr& object)
		: object(object), progress(0)
	{
		if (barTexture == NULL)
		{
			barTexture = new ofImage("data/texturas/Building_texture.jpg");
		}
		if (roofTexture == NULL)
		{
			roofTexture = new ofImage("data/texturas/oba.jpg");
		}
		vector<ofColor> colors;
		for(int i = 0; i < 5; i ++)
			colors.push_back(ofColor(rand()%255,rand()%255,rand()%255));
		vis.setup(colors);
	}

	Box::~Box() {

	}

	void Box::draw(const Floor& floor)
	{
		for (vector<IPolygonPtr>::const_iterator p = object->getPolygons().begin(); p != object->getPolygons().end(); ++p)
		{
			///* DEBUG*/
			switch((*p)->getName())
			{
			case kPolygonNameSideA:
				ofSetColor(255,0,0);
				break;
			case kPolygonNameSideB:
				ofSetColor(0,255,0);
				break;
			case kPolygonNameSideC:
				ofSetColor(0,0,255);
				break;
			case kPolygonNameSideD:
				ofSetColor(255,0,255);
				break;
			case kPolygonNameTop:
				ofSetColor(255,255,255);
				break;
			case kPolygonNameBottom:
				ofSetColor(0,255,255);
				break;
			case kPolygonNameUnknown:
				ofSetColor(255,255,0);
				break;


			}
			ofDrawQuadTextured((*p)->getMathModel().getVertexs(), ofTexCoordsFor(*barTexture));


			//static ofVec3f vA,vB,vC,vD;

			//ofVec3f v3 = (*p)->getMathModel().getVertexs()[0];
			//ofVec3f v0 = (*p)->getMathModel().getVertexs()[1];
			//ofVec3f v1 = (*p)->getMathModel().getVertexs()[2];
			//ofVec3f v2 = (*p)->getMathModel().getVertexs()[3];

			///*	__________
			//	|2		3|
			//	|		 |
			//	|		 |
			//	|1		0|
			//*/

			//if((*p)->getName() == kPolygonNameSideB ||  (*p)->getName() == kPolygonNameSideD) //Side 
			//{
			//	// Bind Texture
			//	ofSetColor(255,255,255,255);
			//	ofTexture bgTex = vis.getBgTexture();
			//	bgTex.bind();
			//
			//	ofVec3f h12 = v2 - v1;
			//	ofVec3f h03 = v3 - v0;			
			//
			//	ofVec3f v3p = v3 - (h03*(1-progress)); //(v3.x, v0.y - progress * h03, v3.z);
			//	ofVec3f v2p = v2 - (h12*(1-progress)); //(v2.x, v1.y - progress * h12, v2.z);
			//	
			//	vector<ofVec2f> texCoords(ofTexCoordsFor(*barTexture));
			//	for (vector<ofVec2f>::iterator t = texCoords.begin(); t != texCoords.end(); ++t)
			//	{
			//		*t *= progress;
			//	}

			//	vector<ofVec3f> vertexs;
			//	vertexs.push_back(v1);
			//	vertexs.push_back(v0);
			//	vertexs.push_back(v3p);
			//	vertexs.push_back(v2p);

			//	ofDrawQuadTextured(vertexs, texCoords);

			//	bgTex.unbind();

			//	ofSetColor(0,0,0);
			//	glBegin(GL_QUADS);      
			//		glVertex3f(v3p.x, v3p.y, v3p.z); 
			//		glVertex3f(v2p.x, v2p.y, v2p.z);
			//		glVertex3f(v2.x, v2.y, v2.z);
			//		glVertex3f(v3.x, v3.y, v3.z);
			//	glEnd();

			//	//vis.draw(v1,v0,v3p,v2p,true);

			//}
			//else if((*p)->getName() == kPolygonNameSideA ) //Front
			//{
			//	vis.draw(v1,v0,v3,v2);
			//}
			//else if((*p)->getName() == kPolygonNameTop) //TOP
			//{
			//	ofSetColor(255,255,255,255);
			//	ofTexture bgTex = vis.getBgTexture();
			//	bgTex.bind();
			//	ofDrawQuadTextured((*p)->getMathModel().getVertexs(), ofTexCoordsFor(bgTex));
			//	bgTex.unbind();
			//}
			////else if((*p)->getName() == kPolygonNameBottom) //TOP
			////{
			////	ofVec3f centroid = computeCentroid((*p)->getMathModel().getVertexs());
			////	ofSetColor(255,255,255,255);
			////	ofPushMatrix();
			////	ofTranslate(centroid);
			////	ofScale(1.5,1.5,1.5);

			////	ofTexture bgTex = vis.getBgTexture();
			////	bgTex.bind();
			////	ofDrawQuadTextured((*p)->getMathModel().getVertexs(), ofTexCoordsFor(bgTex));
			////	bgTex.unbind();
			////}
		}
	}

	void Box::update(float progress) {
		this->progress = progress;
		vis.update();
	}

}
