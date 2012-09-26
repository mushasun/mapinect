#include "Menu.h"

#include "ofGraphicsUtils.h"
#include "ObjectButton.h"
#include "SimpleButton.h"
#include "StoryConstants.h"
#include <cmath>

#include <pcl/common/transforms.h>
#include "transformationUtils.h"

namespace story
{
	ofImage** Menu::buttonTextures = NULL;
	const int kTexturesPerButton = 2;
	ofSoundPlayer*	Menu::ding = NULL;

	

	/*-------------------------------------------------------------*/
	void Menu::setup(IButtonManager* btnManager)
	{
		this->btnManager = btnManager;

		loadSounds();
		loadTextures();
	}

	/*-------------------------------------------------------------*/
	Menu::Menu()
	{
		active = false;
		inAction = false;
		timeMenuShown = -1;
	}

	/*-------------------------------------------------------------*/
	Menu::~Menu()
	{
		if (buttonTextures != NULL)
		{
			for (int i = 0; i < kTexturesPerButton * BUTTON_COUNT; i++)
			{
				delete buttonTextures[i];
			}
			delete buttonTextures;
			buttonTextures = NULL;
		}
	}

	/* Events */
	/*-------------------------------------------------------------*/
	void Menu::buttonEvent(const IButtonPtr& btn, const DataTouch& touchPoint, bool released)
	{
		if (active && released) 
		{
			if(actions.find(btn->getId()) == actions.end())
				return; // no es un boton del menu

			MenuAction action = actions[btn->getId()];
		
			switch(action)
			{
				case STREET:
					StoryStatus::setProperty(ADDING_STREET, true);
					break;
				case RIVER:
					StoryStatus::setProperty(ADDING_RIVER, true);
					break;
				case HOUSE:
					StoryStatus::setProperty(ADDING_HOUSE, true);
					break;
				case POWERPLANT:
					StoryStatus::setProperty(ADDING_POWERPLANT, true);
					break;
				case WATERPLANT:
					StoryStatus::setProperty(ADDING_WATERPLANT, true);
					break;
				case FIRE:
					StoryStatus::setProperty(WANT_TO_BURN, true);
					break;
				case CAMARA_1:
					StoryStatus::setProperty(SET_CAMERA_1, true);
					break;
				case CAMARA_2:
					StoryStatus::setProperty(SET_CAMERA_2, true);
					break;
				case CAMARA_3:
					StoryStatus::setProperty(SET_CAMERA_3, true);
					break;
			}
			
			ding->play();
			if(action != FIRE &&
				action != CAMARA_1 &&
				action != CAMARA_2 &&
				action != CAMARA_3)
			{
				inAction = true;
				removeMenu();

				//Menu Accion
				Polygon3D table = touchPoint.getPolygon()->getMathModel();
				ofVec3f begin = touchPoint.getTouchPoint();
				ofVec3f arriba = ofVec3f(0.f, 0.f, BUTTON_SIDE);
				ofVec3f costado = ofVec3f(BUTTON_SIDE*2, 0.f, 0.f);
				begin -= costado/2;
				vector<ofVec3f> button_vertex;
				//primero casa
				button_vertex.push_back(table.project(begin + arriba));
				button_vertex.push_back(table.project(begin - arriba));
				button_vertex.push_back(table.project(begin - arriba + costado));
				button_vertex.push_back(table.project(begin + arriba + costado));
				Polygon3D area = Polygon3D(button_vertex);
				SimpleButton *okButton = new SimpleButton(area, buttonTextures[(BUTTON_COUNT)*kTexturesPerButton], buttonTextures[((BUTTON_COUNT)*kTexturesPerButton) + 1]);
				actions[okButton->getId()] = FINISH;
				btnManager->addButton(IButtonPtr(okButton));
			}
			else
				removeMenu();
			
		}
		else if (active)
		{
			timeMenuShown = 0;
		}
		else if(inAction)
		{
			if(actions.find(btn->getId()) == actions.end())
				return; // no es un boton del menu

			MenuAction action = actions[btn->getId()];
			if(action == FINISH)
			{
				StoryStatus::setStoryMode(STORY_ACTION_MODE);
				ding->play();
				inAction = false;
				removeMenu();
			}
		}

	}

	/*-------------------------------------------------------------*/
	void Menu::objectEvent(const IObjectPtr& object, const DataTouch& touchPoint)
	{
		if (timeMenuShown <= 0 && !active && !inAction && touchPoint.getType() == kTouchTypeStarted)
		{
			active = true;
			timeMenuShown = 0;			
			//dibujo el menu de cosas a construir
			Polygon3D table = object->getPolygons().at(0)->getMathModel();

			ofVec3f begin = touchPoint.getTouchPoint();
			
			ofVec3f yAxis = table.getPlane().getNormal();
			ofVec3f xAxis = (table.project(ofVec3f(1,0,0))- begin).getNormalized();
			ofVec3f zAxis = yAxis.crossed(xAxis).getNormalized();

			Eigen::Vector3f yEigenAxis(yAxis.x,yAxis.y,yAxis.z);
			float angle = 0;
			float step = TWO_PI / BUTTON_COUNT;
			Eigen::Affine3f rotationMatrix;
			Eigen::Affine3f translationMatrix;
			translationMatrix = Eigen::Translation<float,3>(begin.x,begin.y,begin.z);

			for (int i = 0; i < BUTTON_COUNT; i++)
			{
				ofVec3f c1 = ofVec3f(0,0,0) + (xAxis * (StoryConstants::MENU_RADIUS - BUTTON_SIDE/2.0)) - zAxis * (BUTTON_SIDE/2.0); 
				ofVec3f c2 = c1 + (xAxis * BUTTON_SIDE);
				ofVec3f c3 = c2 + (zAxis * BUTTON_SIDE);
				ofVec3f c4 = c1 + (zAxis * BUTTON_SIDE);
				vector<ofVec3f> button_vertex;
				button_vertex.push_back(c1);
				button_vertex.push_back(c2);
				button_vertex.push_back(c3);
				button_vertex.push_back(c4);

				rotationMatrix = Eigen::AngleAxis<float>(angle, yEigenAxis);
				button_vertex = transformVector(button_vertex, rotationMatrix);
				button_vertex = transformVector(button_vertex, translationMatrix);

				Polygon3D area(button_vertex);
				SimpleButton *button = new SimpleButton(
					area,
					buttonTextures[i * kTexturesPerButton],
					buttonTextures[i * kTexturesPerButton + 1]);
				actions[button->getId()] = (MenuAction)i;
				IButtonPtr buttonPtr(button);
				btnManager->addButton(buttonPtr);
				angle += step;
			}

		}
		else
		{
//			cout << "No entra el menu:" << endl;
//			cout << (timeMenuShown <= 0) << !active << !inAction << endl;
		}
	}

	/*-------------------------------------------------------------*/
	void Menu::update(float elapsedTime)
	{
		if (timeMenuShown >= 0)
		{
			timeMenuShown += elapsedTime;
			if (timeMenuShown > StoryConstants::MENU_LIVE_TIME)
			{
				timeMenuShown = -1;
				if (active) 
				{
					removeMenu();
				}
			}
		}
	}

	/*-------------------------------------------------------------*/
	void Menu::draw()
	{
		
	}

	/*-------------------------------------------------------------*/
	void Menu::removeMenu()
	{
		for(map<int,MenuAction>::iterator it = actions.begin(); it != actions.end(); ++it)
		{
			btnManager->removeButton(it->first);
		}
		actions.clear();
		active = false;
	}

	/* Textures */
	/*-------------------------------------------------------------*/
	void Menu::loadTextures()
	{
		if (buttonTextures == NULL)
		{
			buttonTextures = new ofImage* [BUTTON_COUNT * kTexturesPerButton];
			buttonTextures[0] = new ofImage("data/texturas/menu/road.jpg");
			buttonTextures[1] = new ofImage("data/texturas/menu/roadOn.jpg");
			buttonTextures[2] = new ofImage("data/texturas/menu/river.jpg");
			buttonTextures[3] = new ofImage("data/texturas/menu/riverOn.jpg");
			buttonTextures[4] = new ofImage("data/texturas/menu/house.jpg");
			buttonTextures[5] = new ofImage("data/texturas/menu/houseOn.jpg");
			buttonTextures[6] = new ofImage("data/texturas/menu/power.jpg");
			buttonTextures[7] = new ofImage("data/texturas/menu/powerOn.jpg");
			buttonTextures[8] = new ofImage("data/texturas/menu/water.jpg");
			buttonTextures[9] = new ofImage("data/texturas/menu/waterOn.jpg");
			buttonTextures[10] = new ofImage("data/texturas/menu/fire.png");
			buttonTextures[11] = new ofImage("data/texturas/menu/fireOn.png");
			buttonTextures[12] = new ofImage("data/texturas/menu/CameraIcon1.png");
			buttonTextures[13] = new ofImage("data/texturas/menu/CameraIcon1On.png");
			buttonTextures[14] = new ofImage("data/texturas/menu/CameraIcon2.png");
			buttonTextures[15] = new ofImage("data/texturas/menu/CameraIcon2On.png");
			buttonTextures[16] = new ofImage("data/texturas/menu/CameraIcon3.png");
			buttonTextures[17] = new ofImage("data/texturas/menu/CameraIcon3On.png");
			buttonTextures[18] = new ofImage("data/texturas/menu/ok.png");
			buttonTextures[19] = new ofImage("data/texturas/menu/okOn.png");

		}
	}

	/* Sounds */
	/*-------------------------------------------------------------*/
	void Menu::loadSounds()
	{
		ding = new ofSoundPlayer();
		ding->loadSound("data/sonidos/menu/ding.wav");

	}
}
