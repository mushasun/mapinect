#include "DraggableButton.h"
#include "pointUtils.h"

namespace mapinect {
	DraggableButton::DraggableButton(Polygon3D polygon, ofColor idle, ofColor pressed):SimpleButton(polygon, idle, pressed)
	{
	}


	DraggableButton::~DraggableButton(void)
	{
	}

	ButtonEvent DraggableButton::updateTouchPoints(DataTouch touch)
	{
		map<int,DataTouch> prevContacts = this->getContacts();
		ButtonEvent evnt = SimpleButton::updateTouchPoints(touch);
		map<int,DataTouch> postContacts = this->getContacts();

		if(prevContacts.size() == 1 &&
			postContacts.size() == 1)
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

		return evnt;
	}
}