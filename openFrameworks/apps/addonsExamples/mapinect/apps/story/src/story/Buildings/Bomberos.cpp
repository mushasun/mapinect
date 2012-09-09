#include "bomberos.h"

#include "ofGraphicsUtils.h"
#include "ObjectButton.h"
#include <cmath>
#include "../StoryConstants.h"
#include "../StoryStatus.h"

namespace story
{
	ofImage* Bomberos::txTruckTop = NULL;
	ofImage* Bomberos::txTruckSide = NULL;
	ofImage* Bomberos::txTruckFront = NULL;
	ofImage* Bomberos::txTruckBack = NULL;
	ofSoundPlayer*	Bomberos::water = NULL;
	ofSoundPlayer*	Bomberos::sirena = NULL;

	/*-------------------------------------------------------------*/
	void Bomberos::setup()
	{
		loadSounds();
		loadTextures();
	}

	/*-------------------------------------------------------------*/
	Bomberos::Bomberos(const IObjectPtr& object, IButtonManager* btnManager):Box(object,btnManager)
	{
		timeWatering = 0;
		sirena->setPaused(false);
		associateTextures(object);
	}

	/* Events */
	/*-------------------------------------------------------------*/
	void Bomberos::buttonEvent(const IButtonPtr& btn, bool released)
	{

	}

	/*-------------------------------------------------------------*/
	void Bomberos::objectEvent(const DataTouch& touchPoint, const BuildType& selection)
	{
	}

	/*-------------------------------------------------------------*/
	void Bomberos::update(float elapsedTime)
	{

		if (this->object->getCenter().distance(StoryStatus::getofVec3fProperty(CENTROID_BURNING_HOUSE)) < 0.3)
		{
			//cout << this->object->getCenter().distance(StoryStatus::getofVec3fProperty(CENTROID_BURNING_HOUSE)) << endl;
			//comenzar con el sonido del agua
			if (timeWatering == 0)
			{
				cout << "comienzo a tirar agua" << endl;
				sirena->setVolume(0.3);
				water->setVolume(1.0);
				water->setPaused(false);
			}
			timeWatering += elapsedTime;
			if (timeWatering > 10 && StoryStatus::getProperty(ALREADY_BURNING))
			{
				StoryStatus::setProperty(FIREMAN_FINISHED, true);
				StoryStatus::setProperty(ALREADY_BURNING, false);
				water->setPaused(true);
				sirena->setPaused(true);
				StoryStatus::setStoryMode(STORY_MOVE_AND_ACTION_MODE);
			}

		}
	}

	void Bomberos::draw()
	{
		Box::draw();
		/*ofSetColor(kRGBWhite);

		for (vector<IPolygonPtr>::const_iterator p = object->getPolygons().begin(); p != object->getPolygons().end(); ++p)
		{
			ofImage* texture;
			IPolygonName face = (*p)->getName();
			switch(face)
			{
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
				case kPolygonNameTop:
					texture = textureTop;
				default:
					texture = textureA;
			}

			texture->bind();
			ofDrawQuadTextured((*p)->getMathModel().getVertexs(), ofTexCoordsFor());
			texture->unbind();
		}*/
	}

	/* Textures */
	/*-------------------------------------------------------------*/
	void Bomberos::loadTextures()
	{
		txTruckTop = new ofImage("data/texturas/bomberos/top.png");
		txTruckSide = new ofImage("data/texturas/bomberos/side.png");
		txTruckFront = new ofImage("data/texturas/bomberos/front.png");
		txTruckBack = new ofImage("data/texturas/bomberos/back.png");

	}

	/*-------------------------------------------------------------*/
	void Bomberos::associateTextures(const IObjectPtr& object)
	{
		Polygon3D sideA = object->getPolygon(kPolygonNameSideA)->getMathModel();
		Polygon3D sideB = object->getPolygon(kPolygonNameSideB)->getMathModel();
		float aWidth = (sideA.getVertexs().at(1) - sideA.getVertexs().at(2)).length();
		float bWidth = (sideB.getVertexs().at(1) - sideB.getVertexs().at(2)).length();

		cout << "ladoA: " << aWidth << " - LadoB: " << bWidth <<endl;
		textureA = aWidth > bWidth ? txTruckSide : txTruckFront;
		textureB = aWidth > bWidth ? txTruckFront : txTruckSide;
		textureC = aWidth > bWidth ? txTruckSide : txTruckBack;
		textureD = aWidth > bWidth ? txTruckBack : txTruckSide;
		textureTop = txTruckTop;
	}

	/* Sounds */
	/*-------------------------------------------------------------*/
	void Bomberos::loadSounds()
	{
		water = new ofSoundPlayer();
		water->loadSound("data/sonidos/bomberos/water.wav");
		water->play();
		water->setPaused(true);
		water->setLoop(true);

		sirena = new ofSoundPlayer();
		sirena->loadSound("data/sonidos/bomberos/sirena.wav");
		sirena->play();
		sirena->setPaused(true);
		sirena->setLoop(true);
	}
}
