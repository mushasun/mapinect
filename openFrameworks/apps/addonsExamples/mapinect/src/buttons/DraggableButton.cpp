#include "DraggableButton.h"

#include "ofGraphicsUtils.h"
#include "pointUtils.h"
#include "transformationUtils.h"

namespace mapinect {
	DraggableButton::DraggableButton(Polygon3D polygon, ofColor idle, ofColor pressed)
		: SimpleButton(polygon, idle, pressed),
			translationBase(BAD_OFVEC3F), resizeLineBase(BAD_OFVEC3F, BAD_OFVEC3F)
	{
		setRepeatBehavior(ofTexCoordsFor(), false, false);
		//lastScale = numeric_limits<float>::min();
	}

	DraggableButton::DraggableButton(Polygon3D polygon, ofImage* idle, ofImage* pressed)
		: SimpleButton(polygon, idle, pressed),
			translationBase(BAD_OFVEC3F), resizeLineBase(BAD_OFVEC3F, BAD_OFVEC3F)
	{
		//lastScale = numeric_limits<float>::min();
	}


	DraggableButton::~DraggableButton(void)
	{
	}

	void DraggableButton::setRepeatBehavior(const vector<ofVec2f>& baseTexCoords, bool repeatS, bool repeatT)
	{
		this->baseTexCoords = baseTexCoords;
		this->repeatS = repeatS;
		this->repeatT = repeatT;
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

			ofVec3f leaderPrevPos = prevContacts.find(leaderTouch)->second.getTouchPoint();
			ofVec3f leaderPostPos = postContacts.find(leaderTouch)->second.getTouchPoint();

			if (isTranslating)
			{
				ofVec3f translation = leaderPostPos - leaderPrevPos;

				polygon = transformPolygon3D(polygon, getTranslationMatrix(translation));
			}
		
			if (leaderTouch != touch.getId())
			{
				ofVec3f followerPostPos = postContacts.find(touch.getId())->second.getTouchPoint();
				if (willResize)
				{
					resizeLineBase = Line3D(leaderPostPos, followerPostPos);
					resizePolygonBase = Polygon3D(polygon);

					const float angleArea = 20.0f;
					float angle = resizePolygonBase.getEdges()[0].getDirection()
						.angle(resizeLineBase.getDirection());
					if (inRange(angle, -angleArea, +angleArea))
						resizeScalingDirection = kHorizontalScaling;
					else if (inRange(90 - angle, -angleArea, +angleArea))
						resizeScalingDirection = kVerticalScaling;
					else
						resizeScalingDirection = kUniformScaling;

				}
				else if (isResizing)
				{
					Line3D currentLine(leaderPostPos, followerPostPos);
					
					float rotation = resizeLineBase.getDirection().angle(currentLine.getDirection());
					const float rotationLimit = 1.0f;

					float scale = currentLine.segmentLength() / resizeLineBase.segmentLength();
					cout << "scale: " << scale << endl;
					const int sAxisIx = 0;
					const int tAxisIx = 3;
					const float scaleLimit = 1.0f;

					{
						
						setTexCoords(ofTexCoordsFor(
							repeatS ? scale : 1.0f,
							repeatT ? scale : 1.0f));
						
					}
					{
						Eigen::Affine3f transformation = getTranslationMatrix(resizePolygonBase.getCentroid());
						if (abs(scale - 1.0f) < scaleLimit)
						{
							if (resizeScalingDirection == kUniformScaling)
							{
								cout << "escalamiento uniforme" << endl;
								transformation = transformation * getScaleMatrix(scale);
							}
							else
							{
								const ofVec3f xAxis(1, 0, 0);
								const ofVec3f yAxis(0, 1, 0);
								const ofVec3f zAxis(0, 0, 1);
								const Line3D& sAxis(resizePolygonBase.getEdges()[sAxisIx]);
								const ofVec3f uAxis(resizePolygonBase.getPlane().getNormal());
								const float xAngle = sAxis.getDirection().angleRad(xAxis);
								const float yAngle = uAxis.angleRad(yAxis);
								transformation = transformation * getRotationMatrix(xAxis, yAngle);
								transformation = transformation * getRotationMatrix(yAxis, xAngle);
								if (resizeScalingDirection == kHorizontalScaling)
								{
									cout << "escalamiento horizontal" << endl;
									transformation = transformation * getScaleMatrix(ofVec3f(scale,1.0f,1.0f));
								}
								else if (resizeScalingDirection == kVerticalScaling)
								{
									cout << "escalamiento vertical" << endl;
									transformation = transformation * getScaleMatrix(ofVec3f(1.0f,1.0f,scale));
								}
								transformation = transformation * getRotationMatrix(yAxis, -xAngle);
								transformation = transformation * getRotationMatrix(xAxis, -yAngle);
							}
						}
						if (abs(rotation) > rotationLimit)
						{
							transformation = transformation * getRotationMatrix(
								resizePolygonBase.getPlane().getNormal(),
								ofDegToRad(-rotation));
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
			/*
			if(isResizing && leaderTouch != touch.getId())
			{
				map<int,DataTouch>::iterator leaderTouchIter = postContacts.find(leaderTouch);
				map<int,DataTouch>::iterator postTouchIter = postContacts.find(touch.getId());
				if (postContacts.count(leaderTouch) > 0 &&
					postContacts.count(touch.getId()) > 0)
				{
					ofVec3f leaderTouch = leaderTouchIter->second.getTouchPoint();
					ofVec3f postTouch = postTouchIter->second.getTouchPoint();
					ofVec3f scaleVector = postTouch - leaderTouch;
					float scale = abs(scaleVector.length());

					if(lastScale != numeric_limits<float>::min() &&
						fabs(lastScale - scale) > 0.008)
					{		
						scaleVector.x = abs(scaleVector.x);
						scaleVector.y = abs(scaleVector.y);
						scaleVector.z = abs(scaleVector.z);

						ofVec3f unit = ofVec3f(1.0,1.0,1.0);

						ofVec3f scaleFactor = lastScale < scale ? unit + scaleVector : unit - scaleVector;
						cout << scale << endl;
					
						const int sLine = 0;
						const int tLine = 3;
						float sLen = polygon.getEdges()[sLine].segmentLength();
						float tLen = polygon.getEdges()[tLine].segmentLength();

						polygon = transformPolygon3D(polygon,
							getTranslationMatrix(polygon.getCentroid()) *
							getScaleMatrix(scaleFactor) *
							getTranslationMatrix(-polygon.getCentroid()));

					}
					lastScale = scale;


				}
				*/
			}
			/*
			if(postContacts.size() != 2)
			{
				lastScale = numeric_limits<float>::min();
			}
			*/
		return evnt;
	}
}