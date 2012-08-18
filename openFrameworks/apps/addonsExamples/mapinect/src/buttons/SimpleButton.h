#ifndef SIMPLE_BUTTON_H__
#define SIMPLE_BUTTON_H__

#include "BaseButton.h"

namespace mapinect
{
	class SimpleButton;
	typedef boost::shared_ptr<SimpleButton> SimpleButtonPtr;

	class SimpleButton : public BaseButton
	{
	public:
		SimpleButton();
		SimpleButton(const Polygon3D& polygon, const ofColor& idle, const ofColor& pressed);
		SimpleButton(const Polygon3D& polygon, ofImage* idle, ofImage* pressed);
		virtual void draw();
		virtual ButtonEvent updateTouchPoints(const DataTouch& touch);
		inline virtual vector<ofVec3f>		getVertexs() { return polygon.getVertexs(); }
	protected:
		virtual bool isInTouch(const DataTouch& touch);
		Polygon3D polygon;
	private:
		void init();
	};
}

#endif //SIMPLE_BUTTON_H__