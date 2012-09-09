#include "Box.h"

#include "ofGraphicsUtils.h"
#include "StoryConstants.h"
#include "StoryStatus.h"

namespace story
{
	vector<ofImage*> Box::fireImgs;

	Box::Box(const IObjectPtr& object, IButtonManager* btnManager)
		: object(object)
	{
		fireSpriteA = new AnimatedSprite(fireImgs,0.07,object->getPolygon(kPolygonNameSideA)->getMathModel());
		fireSpriteB = new AnimatedSprite(fireImgs,0.07,object->getPolygon(kPolygonNameSideB)->getMathModel());
		fireSpriteC = new AnimatedSprite(fireImgs,0.07,object->getPolygon(kPolygonNameSideC)->getMathModel());
		fireSpriteD = new AnimatedSprite(fireImgs,0.07,object->getPolygon(kPolygonNameSideD)->getMathModel());

		isBurning = false;
	}

	Box::~Box()
	{
		delete(fireSpriteA);
		delete(fireSpriteB);
		delete(fireSpriteC);
		delete(fireSpriteD);
		object.reset();
	}

	void Box::setup()
	{
		for(int i = 0; i < 104; i++)
		{
			ofImage* img = new ofImage("data/texturas/fire/f" + ofToString(i) + ".png");
			fireImgs.push_back(img);
		}
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
			ofDrawQuadTextured((*p)->getMathModel().getVertexs(), ofTexCoordsFor());
			texture->unbind();

			
			
			//ofSetColor(kRGBWhite);
			//glLineWidth(0.01);
			//	for(int i = 0; i < (*p)->getMathModel().getVertexs().size() ; i ++)
			//	{
			//		ofVec3f v = (*p)->getMathModel().getVertexs().at(i);
			//		/*ofCircle(v.x,v.y,v.z, 0.005);*/
			//		ofVec3f c = (*p)->getMathModel().getCentroid();
			//		glBegin(GL_LINES); 
			//		glVertex3f(c.x,c.y,c.z);
			//		ofVec3f pointer = c + (*p)->getMathModel().getPlane().getNormal();
			//		glVertex3f( pointer.x,pointer.y,pointer.z); 
			//		glEnd(); 
			//	}
			//ofSetColor(kRGBGreen);
			//ofCircle((*p)->getMathModel().getCentroid().x,(*p)->getMathModel().getCentroid().y,(*p)->getMathModel().getCentroid().z,
			//				0.005);

			//ofSetColor(kRGBWhite);
		}
		if(isBurning)
		{
			fireSpriteA->draw();
			fireSpriteB->draw();
			fireSpriteC->draw();
			fireSpriteD->draw();
		}
	}

	void Box::updateModelObject(const IObjectPtr& ob)
	{
		object = ob;
		fireSpriteA->setPolygon(object->getPolygon(kPolygonNameSideA)->getMathModel());
		fireSpriteB->setPolygon(object->getPolygon(kPolygonNameSideB)->getMathModel());
		fireSpriteC->setPolygon(object->getPolygon(kPolygonNameSideC)->getMathModel());
		fireSpriteD->setPolygon(object->getPolygon(kPolygonNameSideD)->getMathModel());
	}

	void Box::update(float elapsedTime)
	{
		if (isBurning)
		{
			fireSpriteA->update(elapsedTime);
			fireSpriteB->update(elapsedTime);
			fireSpriteC->update(elapsedTime);
			fireSpriteD->update(elapsedTime);
			if(StoryStatus::getProperty(FIREMAN_FINISHED))
			{
				burnSound->stop();
				delete burnSound;
				isBurning = false;
				StoryStatus::setProperty(FIREMAN_FINISHED,false);
				StoryStatus::setProperty(BURNING, false);
			}
		}
	}

	void Box::stopBurn()
	{

	}

	void Box::burn()
	{
		if (!StoryStatus::getProperty(ALREADY_BURNING)) //para evitar tener 2 casas ardiendo
		{
			burnSound = new ofSoundPlayer();
			burnSound->loadSound("data/sonidos/house/burning.wav");
			burnSound->play();
			burnSound->setLoop(true);
			StoryStatus::setProperty(CENTROID_BURNING_HOUSE, this->object->getCenter());
			StoryStatus::setProperty(ALREADY_BURNING, true);
			isBurning = true;
		}
	}
}
