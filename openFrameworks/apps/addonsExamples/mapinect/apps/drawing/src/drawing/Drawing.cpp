#include "Drawing.h"

#include "DraggableButton.h"
#include "ofGraphicsUtils.h"
#include "transformationUtils.h"

namespace drawing
{
	const int kCanvasWidth = 512;
	const int kCanvasHeight = 512;
	const float kPenSize = 4.0f;
	
	const float kButtonElevation = 0.001;
	const int kButtons = 3;
	const int kPaletteColors = 6;

	const float kMenuTime = 10.0f;

	Drawing::Drawing()
		: canvas(NULL), menuTimer(0.0f), menuVisible(false), paletteVisible(false)
	{
		object.reset();
	}

	Drawing::~Drawing()
	{
	}

	void Drawing::setup()
	{
		setAppStatus(kAppStatusDrawing);

		menuSound.loadSound("sounds/ding.wav");

		int tx = 0;
		textures.resize((kButtons + 1) * 2);
		textures[tx++] = new ofImage("textures/off0001.png");
		textures[tx++] = new ofImage("textures/on0001.png");
		textures[tx++] = new ofImage("textures/off0002.png");
		textures[tx++] = new ofImage("textures/on0002.png");
		textures[tx++] = new ofImage("textures/off0003.png");
		textures[tx++] = new ofImage("textures/on0003.png");
		textures[tx++] = new ofImage("textures/off0004.png");
		textures[tx++] = new ofImage("textures/on0004.png");

		int c = 0;
		paletteColors.resize(kPaletteColors);
		paletteColors[c++] = kRGBBlack;
		paletteColors[c++] = kRGBBlue;
		paletteColors[c++] = kRGBRed;
		paletteColors[c++] = kRGBGreen;
		paletteColors[c++] = kRGBYellow;
		paletteColors[c++] = kRGBWhite;

		c = 0;
		paletteTextures.resize(kPaletteColors * 2);
		paletteTextures[c++] = new ofImage("textures/off0005.png");
		paletteTextures[c++] = new ofImage("textures/on0005.png");
		paletteTextures[c++] = new ofImage("textures/off0006.png");
		paletteTextures[c++] = new ofImage("textures/on0006.png");
		paletteTextures[c++] = new ofImage("textures/off0007.png");
		paletteTextures[c++] = new ofImage("textures/on0007.png");
		paletteTextures[c++] = new ofImage("textures/off0008.png");
		paletteTextures[c++] = new ofImage("textures/on0008.png");
		paletteTextures[c++] = new ofImage("textures/off0009.png");
		paletteTextures[c++] = new ofImage("textures/on0009.png");
		paletteTextures[c++] = new ofImage("textures/off0010.png");
		paletteTextures[c++] = new ofImage("textures/on0010.png");
	}

	void Drawing::update(float elapsedTime)
	{
		if (canvas != NULL &&
			status != kAppStatusFollowingObject &&
			status != kAppStatusPickingColor)
		{
			menuTimer += elapsedTime;
			if (menuTimer >= kMenuTime)
			{
				createMenu(table.getCentroid());
			}
		}
	}

	void Drawing::draw()
	{
		if (canvas != NULL)
			canvas->draw();

		if (object.get() != NULL)
			for (int i = 0; i < object->getPolygons().size(); i++)
			{
				ofSetColor(kRGBBlue + ofColor(20 * i, 20 * i, 0));
				ofDrawQuad(object->getPolygons()[i]->getMathModel().getVertexs());
			}

		for (map<int, DataTouch>::const_iterator it = touchPoints.begin(); it != touchPoints.end(); ++it)
		{
			ofSetColor(kRGBGreen);
			ofVec3f s = it->second.getTouchPoint();
			ofDrawCircle(s, 0.01);
		}
	}

	void Drawing::objectDetected(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			table = object->getPolygons()[0]->getMathModel();
			Polygon3D tableCanvas(transformPolygon3D(table,
				getTranslationMatrix(table.getPlane().getNormal() * -0.0001)));
			canvas = new Canvas(
				object->getId(),
				tableCanvas,
				kCanvasWidth, kCanvasHeight,
				kRGBWhite, kRGBRed,
				kPenSize);
		}
		else if (kAppStatusFollowingObject)
		{
			if (this->object == NULL || this->object->getId() != object->getId())
			{
				this->object = object;
				armController->followObject(this->object);
			}
		}
	}

	void Drawing::objectUpdated(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			table = object->getPolygons()[0]->getMathModel();
			Polygon3D tableCanvas(transformPolygon3D(table,
				getTranslationMatrix(table.getPlane().getNormal() * -0.0001)));
			canvas->update(tableCanvas);
		}
		else if (this->object.get() != NULL
			&& object->getId() == this->object->getId())
		{
			this->object = object;
		}
	}

	void Drawing::objectLost(const IObjectPtr& object)
	{
		if (this->object.get() != NULL && this->object->getId() == object->getId())
		{
			this->object.reset();
			if (status == kAppStatusFollowingObject)
				setAppStatus(kAppStatusDrawing);
		}
	}

	void Drawing::objectMoved(const IObjectPtr& object, const DataMovement& movement)
	{
	}

	void Drawing::objectTouched(const IObjectPtr& object, const DataTouch& touchPoint)
	{
		if (status == kAppStatusDrawing)
		{
			if (canvas != NULL)
			{
				destroyMenu();
				canvas->touchEvent(touchPoint);
			}
		}
		else if (status == kAppStatusInsertingPicture)
		{
			if (object->getId() == TABLE_ID)
			{
				createPicture(touchPoint.getTouchPoint());
				setAppStatus(kAppStatusPositioningPicture);
			}
		}
	}

	void Drawing::pointTouched(const DataTouch& touchPoint)
	{
		map<int, DataTouch>::iterator it = touchPoints.find(touchPoint.getId());
		if (it == touchPoints.end())
		{
			if (touchPoint.getType() != kTouchTypeReleased)
				touchPoints.insert(make_pair(touchPoint.getId(), touchPoint));
		}
		else if (touchPoint.getType() == kTouchTypeReleased)
		{
			touchPoints.erase(it);
		}
		else
		{
			it->second = touchPoint;
		}
	}

	void Drawing::buttonPressed(const IButtonPtr& button, const DataTouch& touchPoint)
	{
		map<int, int>::const_iterator iter = actions.find(button->getId());
		if (iter == actions.end())
		{
			destroyMenu();
		}
	}

	void Drawing::buttonReleased(const IButtonPtr& button, const DataTouch& touchPoint)
	{
		map<int, int>::const_iterator iter = actions.find(button->getId());
		if (iter != actions.end())
		{
			menuSound.play();
			if (menuVisible)
			{
				AppAction action = (AppAction)(iter->second);
				if (action == kAppActionPickColor)
				{
					setAppStatus(kAppStatusPickingColor);
					createPalette(touchPoint.getTouchPoint());
				}
				else if (action == kAppActionFollowObject)
				{
					setAppStatus(kAppStatusFollowingObject);
				}
				else if (action == kAppActionInsertPicture)
				{
					setAppStatus(kAppStatusInsertingPicture);
				}
				else if (action == kAppActionConfirmPicture)
				{
					confirmPicture();
					setAppStatus(kAppStatusDrawing);
				}
			}
			else if (paletteVisible)
			{
				canvas->setForeColor(paletteColors[iter->second]);
				setAppStatus(kAppStatusDrawing);
			}
		}
	}

	void Drawing::setAppStatus(AppStatus newStatus)
	{
		if (status != newStatus)
		{
			if (status == kAppStatusPickingColor)
			{
				destroyPalette();
			}
			else if (status == kAppStatusFollowingObject)
			{
			}
			else
			{
				if (status == kAppStatusDrawing)
					if (canvas != NULL)
						canvas->endAllTraces();
				destroyMenu();
			}

			status = newStatus;

			if (status == kAppStatusDrawing)
			{
			}
			else if (status == kAppStatusPickingColor)
			{
			}
			else if (status == kAppStatusFollowingObject)
			{
			}
			else if (status == kAppStatusInsertingPicture)
			{
			}
			else if (status == kAppStatusPositioningPicture)
			{
			}

			if (status == kAppStatusFollowingObject)
			{
				modeManager->enableObjectTracking();
				modeManager->disableTouchTracking();
			}
			else
			{
				modeManager->enableTouchTracking();
				modeManager->disableObjectTracking();
			}
		}
	}

	vector<ofVec3f> Drawing::polygonOnTable(const ofVec3f& center, float xLength, float zLength, float elevation, float radius, float rotationAngle)
	{
		ofVec3f yAxis = table.getPlane().getNormal();
		ofVec3f xAxis = (table.getVertexs()[3] - table.getVertexs()[0]).getNormalized();
		ofVec3f zAxis = yAxis.crossed(xAxis).getNormalized();

		ofVec3f c1 = ofVec3f(0,0,0)
			+ xAxis * (radius - xLength * 0.5f)
			- zAxis * (zLength * 0.5f); 
		ofVec3f c2 = c1 + (xAxis * xLength);
		ofVec3f c3 = c2 + (zAxis * zLength);
		ofVec3f c4 = c1 + (zAxis * zLength);
		vector<ofVec3f> vertexs;
		vertexs.push_back(c1);
		vertexs.push_back(c2);
		vertexs.push_back(c3);
		vertexs.push_back(c4);

		vertexs = transformVector(vertexs, getRotationMatrix(yAxis, rotationAngle));
		vertexs = transformVector(vertexs, getTranslationMatrix(center));
		vertexs = transformVector(vertexs, getTranslationMatrix(yAxis * elevation));

		return vertexs;
	}

	void Drawing::clearActions()
	{
		for (map<int, int>::iterator it = actions.begin(); it != actions.end(); ++it)
			btnManager->removeButton(it->first);
		actions.clear();
	}

	void Drawing::createMenu(const ofVec3f& center)
	{
		if (!menuVisible)
		{
			menuVisible = true;
			
			const float kSideLength = 0.1f;
			const float kSpacing = kSideLength;

			if (status == kAppStatusPositioningPicture)
			{
				vector<ofVec3f> buttonVertexs(polygonOnTable(center, kSideLength, kSideLength, kButtonElevation, 0, 0));
				Polygon3D area(buttonVertexs);
				SimpleButton *button = new SimpleButton(
					area,
					textures[kAppActionConfirmPicture * 2],
					textures[kAppActionConfirmPicture * 2 + 1]);
				actions[button->getId()] = kAppActionConfirmPicture;
				IButtonPtr buttonPtr(button);
				btnManager->addButton(buttonPtr);
			}
			else
			{
				float angle = 0;
				float step = TWO_PI / (float)kButtons;

				for (int i = 0; i < kButtons; i++)
				{
					vector<ofVec3f> buttonVertexs(polygonOnTable(center, kSideLength, kSideLength, kButtonElevation, kSpacing, angle));

					Polygon3D area(buttonVertexs);
					SimpleButton *button = new SimpleButton(
						area,
						textures[i * 2],
						textures[i * 2 + 1]);
					actions[button->getId()] = i;
					IButtonPtr buttonPtr(button);
					btnManager->addButton(buttonPtr);
					angle += step;
				}
			}
		}
	}

	void Drawing::destroyMenu()
	{
		menuTimer = 0.0f;
		if (menuVisible)
		{
			menuVisible = false;
			clearActions();
		}
	}

	void Drawing::createPalette(const ofVec3f& center)
	{
		if (!paletteVisible)
		{
			paletteVisible = true;

			const float kSideLength = 0.1f;
			const float kSpacing = kSideLength * 1.5f;

			float angle = 0;
			float step = TWO_PI / (float)kPaletteColors;

			for (int i = 0; i < kPaletteColors; i++)
			{
				vector<ofVec3f> buttonVertexs(polygonOnTable(center, kSideLength, kSideLength, kButtonElevation, kSpacing, angle));

				Polygon3D area(buttonVertexs);
				SimpleButton *button = new SimpleButton(
					area,
					paletteTextures[i * 2],
					paletteTextures[i * 2 + 1]);
				actions[button->getId()] = i;
				IButtonPtr buttonPtr(button);
				btnManager->addButton(buttonPtr);
				angle += step;
			}
		}
	}

	void Drawing::destroyPalette()
	{
		if (paletteVisible)
		{
			paletteVisible = false;
			clearActions();
		}
	}

	void Drawing::createPicture(const ofVec3f& p)
	{
		static int imageCounter = 0;
		string imagePath = "textures/image" + ofToString(imageCounter) + ".jpg";
		imageCounter = (imageCounter + 1) % 5;
		ofImage* image = new ofImage(imagePath);
		
		const float kSize = 0.2f;
		vector<ofVec3f> vertexs(polygonOnTable(p, kSize, kSize, 0, 0, 0));

		Polygon3D polygon(vertexs);
		
		DraggableButton* button = new DraggableButton(polygon, image, image);
		IButtonPtr buttonPtr(button);
		btnManager->addButton(buttonPtr);

		pictureCurrent = buttonPtr;
		pictureTextureCurrent = image;
	}

	void Drawing::confirmPicture()
	{
		const vector<ofVec3f>& vertexs(pictureCurrent->getVertexs());
		canvas->drawTexture(*pictureTextureCurrent, vertexs);
		btnManager->removeButton(pictureCurrent->getId());
		pictureTextureCurrent = NULL;
		pictureCurrent.reset();
	}

}
