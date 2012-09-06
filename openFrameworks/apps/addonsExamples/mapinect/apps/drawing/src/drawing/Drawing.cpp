#include "Drawing.h"

#include "DraggableButton.h"
#include "ObjectButton.h"
#include "ofGraphicsUtils.h"

namespace drawing
{
	const int kCanvasWidth = 512;
	const int kCanvasHeight = 512;
	const float kPenSize = 4.0f;
	const int kButtons = 2;

	enum AppAction
	{
		kAppActionFollowObject = 0,
		kAppActionInsertPicture
	};

	static vector<ofImage*> textures;
	static map<int, AppAction> buttonsToAction;

	Drawing::Drawing()
		: canvas(NULL), status(kAppStatusDrawing)
	{
		object.reset();
	}

	Drawing::~Drawing()
	{
	}

	void Drawing::setup()
	{
		textures.resize(kButtons * 2);
		textures[0] = new ofImage("textures/followOff.png");
		textures[1] = new ofImage("textures/followOn.png");
		textures[2] = new ofImage("textures/pictureOff.png");
		textures[3] = new ofImage("textures/pictureOn.png");
	}

	void Drawing::update(float elapsedTime)
	{
	}

	void Drawing::draw()
	{
		if (canvas != NULL)
			canvas->draw();
	}

	void Drawing::objectDetected(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			table = object->getPolygons()[0]->getMathModel();
			canvas = new Canvas(
				object->getId(),
				object->getPolygons()[0]->getMathModel(),
				kCanvasWidth, kCanvasHeight,
				kRGBWhite, kRGBRed,
				kPenSize);
		}
		else if (this->object.get() == NULL)
		{
			setAppStatus(kAppStatusMenuPopup);
			this->object = object;
		}
	}

	void Drawing::objectUpdated(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			canvas->update(object->getPolygons()[0]->getMathModel());
		}
		else
		{
			setAppStatus(kAppStatusMenuPopup);
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
				if (touchPoint.getType() == kTouchTypeStarted)
					canvas->setForeColor(ofRandomColor());
				canvas->touchEvent(touchPoint);
			}
		}
		else if (status == kAppStatusInsertingPicture)
		{
			if (object->getId() == TABLE_ID)
			{
				createPicture(touchPoint);
				setAppStatus(kAppStatusDrawing);
			}
			else if (object->getId() == this->object->getId())
			{
				setAppStatus(kAppStatusMenuPopup);
			}
		}
	}

	void Drawing::buttonReleased(const IButtonPtr& button, const DataTouch& touchPoint)
	{
		if (status == kAppStatusMenuPopup)
		{
			map<int, AppAction>::const_iterator iter = buttonsToAction.find(button->getId());
			if (iter != buttonsToAction.end())
			{
				AppAction action = iter->second;
				if (action == kAppActionFollowObject)
				{
					armController->followObject(object);
					setAppStatus(kAppStatusFollowingObject);
				}
				else if (action == kAppActionInsertPicture)
				{
					setAppStatus(kAppStatusInsertingPicture);
				}
			}
		}
	}

	void Drawing::setAppStatus(AppStatus newStatus)
	{
		if (status != newStatus)
		{
			if (status == kAppStatusDrawing)
			{
				if (canvas != NULL)
					canvas->endAllDrawers();
			}
			else if (status == kAppStatusMenuPopup)
			{
				buttonsToAction.clear();
			}
			else if (status == kAppStatusFollowingObject)
			{

			}
			else if (status == kAppStatusInsertingPicture)
			{
				
			}

			status = newStatus;

			if (status == kAppStatusDrawing)
			{
			}
			else if (status == kAppStatusMenuPopup)
			{
				createMenu();
			}
			else if (status == kAppStatusFollowingObject)
			{
			}
			else if (status == kAppStatusInsertingPicture)
			{
			}
		}
	}

	void Drawing::createMenu()
	{
		const float kSideLength = 0.1f;
		for (int i = 0; i < kButtons; i++)
		{
			IButton* button = new ObjectButton(
				object,
				kPolygonNameSideA,
				true,
				textures[i+0],
				textures[i+1],
				kSideLength, kSideLength,
				-kSideLength * 0.5f * (float)i, kSideLength * 0.5f);
			IButtonPtr buttonPtr(button);
			buttonsToAction.insert(make_pair(button->getId(), (AppAction)i));
			btnManager->addButton(buttonPtr);
		}
	}

	void Drawing::createPicture(const DataTouch& p)
	{
		static int imageCounter = 0;
		string imagePath = "textures/image" + ofToString(imageCounter) + ".jpg";
		imageCounter = (imageCounter + 1) % 5;
		ofImage* image = new ofImage(imagePath);
		
		const float kSize = 0.1f;
		ofVec3f normal = table.getPlane().getNormal();
		ofVec3f d = table.project(p.getTouchPoint() + ofVec3f(1,0,0)) - p.getTouchPoint();
		d.normalize();
		ofVec3f c = normal.cross(d).getNormalized();
		vector<ofVec3f> imageVertexs;
		imageVertexs.push_back(p.getTouchPoint() + d * kSize * 0.5f - c * kSize * 0.5f);
		imageVertexs.push_back(p.getTouchPoint() + d * kSize * 0.5f + c * kSize * 0.5f);
		imageVertexs.push_back(p.getTouchPoint() - d * kSize * 0.5f + c * kSize * 0.5f);
		imageVertexs.push_back(p.getTouchPoint() - d * kSize * 0.5f - c * kSize * 0.5f);

		Polygon3D polygon(imageVertexs);
		
		DraggableButton* button = new DraggableButton(polygon, image, image);
		IButtonPtr buttonPtr(button);
		btnManager->addButton(buttonPtr);
	}

}
