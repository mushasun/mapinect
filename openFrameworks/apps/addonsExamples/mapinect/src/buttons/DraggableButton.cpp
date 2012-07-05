#include "DraggableButton.h"
#include "pointUtils.h"

namespace mapinect {
	DraggableButton::DraggableButton(Polygon3D polygon, ofColor idle, ofColor pressed):SimpleButton(polygon, idle, pressed)
	{
		lastScale = numeric_limits<float>::min();
	}

	DraggableButton::DraggableButton(Polygon3D polygon, ofImage* idle, ofImage* pressed):SimpleButton(polygon, idle, pressed)
	{
		lastScale = numeric_limits<float>::min();
	}


	DraggableButton::~DraggableButton(void)
	{
	}

	ButtonEvent DraggableButton::updateTouchPoints(const DataTouch& touch)
	{
		map<int,DataTouch> prevContacts = this->getContacts();
		ButtonEvent evnt = SimpleButton::updateTouchPoints(touch);
		map<int,DataTouch> postContacts = this->getContacts();

		//cout << "t: " << touch.getId() << " l: " << leaderTouch << endl;

		if(prevContacts.size() >= 1 &&
			postContacts.size() >= 1 &&
			!leaderChanged &&
			leaderTouch == touch.getId())
		{
			map<int,DataTouch>::iterator prevTouchIter = prevContacts.find(touch.getId());
			map<int,DataTouch>::iterator postTouchIter = postContacts.find(touch.getId());

			if (prevTouchIter != prevContacts.end() &&
				postTouchIter != postContacts.end())
			{
				ofVec3f prevTouch = prevTouchIter->second.getTouchPoint();
				ofVec3f postTouch = postTouchIter->second.getTouchPoint();

				ofVec3f translation = postTouch - prevTouch;

				Eigen::Affine3f traslation;
				traslation = Eigen::Translation<float,3>(translation.x,translation.y,translation.z);

				polygon = transformPolygon3D(polygon, traslation);
			}
		}
		
		if(prevContacts.size() == 2 &&
				postContacts.size() == 2 &&
				!leaderChanged &&
				leaderTouch != touch.getId())
		{
			map<int,DataTouch>::iterator leaderTouchIter = postContacts.find(leaderTouch);
			map<int,DataTouch>::iterator postTouchIter = postContacts.find(touch.getId());
			if (leaderTouchIter != prevContacts.end() &&
				postTouchIter != postContacts.end())
			{
				ofVec3f leaderTouch = leaderTouchIter->second.getTouchPoint();
				ofVec3f postTouch = postTouchIter->second.getTouchPoint();
				float scale = (postTouch - leaderTouch).length();

				if(lastScale != numeric_limits<float>::min() &&
					fabs(lastScale - scale) > 0.008)
				{		
					float scaleFactor = lastScale < scale ? 1.1 : 0.9;
					cout << scale << endl;

					Eigen::Affine3f scaleMatrix;
					scaleMatrix = Eigen::Scaling<float>(scaleFactor,scaleFactor,scaleFactor);
					polygon = transformPolygon3D(polygon, scaleMatrix);
				}
				lastScale = scale;


			}
		}

		if(postContacts.size() != 2)
		{
			lastScale = numeric_limits<float>::min();
		}

		return evnt;
	}
}