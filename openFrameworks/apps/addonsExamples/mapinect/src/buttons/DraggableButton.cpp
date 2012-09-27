#include "DraggableButton.h"

#include "ofGraphicsUtils.h"
#include "pointUtils.h"
#include "transformationUtils.h"

namespace mapinect {
	DraggableButton::DraggableButton(Polygon3D polygon, ofColor idle, ofColor pressed)
		: SimpleButton(polygon, idle, pressed),
			translationBase(BAD_OFVEC3F), resizeLineBase(BAD_OFVEC3F, BAD_OFVEC3F)
	{
	}

	DraggableButton::DraggableButton(Polygon3D polygon, ofImage* idle, ofImage* pressed)
		: SimpleButton(polygon, idle, pressed),
			translationBase(BAD_OFVEC3F), resizeLineBase(BAD_OFVEC3F, BAD_OFVEC3F)
	{
	}

	DraggableButton::~DraggableButton(void)
	{
	}

	ButtonEvent DraggableButton::updateTouchPoints(const IObjectPtr& object, const DataTouch& touch)
	{
		map<int,DataTouch> prevContacts = this->getContacts();
		ButtonEvent evnt = SimpleButton::updateTouchPoints(object, touch);
		map<int,DataTouch> postContacts = this->getContacts();

		if (!leaderChanged)
		{

			isTranslating = prevContacts.size() == 1 && postContacts.size() == 1;

			bool wasResizing = isResizing;
			bool willResize = prevContacts.size() == 1 && postContacts.size() == 2;
			isResizing = prevContacts.size() == 2 && postContacts.size() == 2;

			if (isTranslating)
			{
				ofVec3f leaderPrevPos = prevContacts.find(leaderTouch)->second.getTouchPoint();
				ofVec3f leaderPostPos = postContacts.find(leaderTouch)->second.getTouchPoint();

				ofVec3f translation = leaderPostPos - leaderPrevPos;

				polygon = transformPolygon3D(polygon, getTranslationMatrix(translation));
			}
		
			if (leaderTouch != touch.getId())
			{
				ofVec3f leaderPostPos = postContacts.find(leaderTouch)->second.getTouchPoint();

				ofVec3f followerPostPos = postContacts.find(touch.getId())->second.getTouchPoint();
				if (willResize)
				{
					resizeLineBase = Line3D(leaderPostPos, followerPostPos);
					resizePolygonBase = Polygon3D(polygon);
				}
				else if (isResizing)
				{
					Line3D currentLine(leaderPostPos, followerPostPos);
					
					float rotation = resizeLineBase.getDirection().angle(currentLine.getDirection());
					const float rotationLimit = 1.0f;

					float scale = currentLine.segmentLength() / resizeLineBase.segmentLength();
					const int sAxisIx = 0;
					const int tAxisIx = 3;
					const float scaleLimit = 1.0f;

					{
						Eigen::Affine3f transformation = getTranslationMatrix(resizePolygonBase.getCentroid());
						if (abs(scale - 1.0f) < scaleLimit)
						{
							transformation = transformation * getScaleMatrix(scale);
						}
						if (abs(rotation) > rotationLimit)
						{
							/*transformation = transformation * getRotationMatrix(
								resizePolygonBase.getPlane().getNormal(),
								ofDegToRad(-rotation));*/
						}
						transformation = transformation * getTranslationMatrix(-resizePolygonBase.getCentroid());

						polygon = transformPolygon3D(resizePolygonBase, transformation);
					}
				}
				else if (wasResizing)
				{
					// nothing to do here
				}
			}
		}
		return evnt;
	}
}