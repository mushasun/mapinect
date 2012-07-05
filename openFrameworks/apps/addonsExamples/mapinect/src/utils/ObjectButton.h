#ifndef OBJECT_BUTTON_H__
#define OBJECT_BUTTON_H__

#include "BaseButton.h"
#include "IObject.h"
#include <set>

namespace mapinect {
	enum OButtonMode{
		kInFace,
		kInFloor,
		kFullObject
	};

	class ObjectButton;
	typedef boost::shared_ptr<ObjectButton> ObjectButtonPtr;

	class ObjectButton : public BaseButton{
	public:
		ObjectButton(const IObjectPtr& obj, ofColor idle, ofColor pressed);
		ObjectButton(const IObjectPtr& obj, ofImage* idle, ofImage* pressed);
		ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor, ofColor idle, ofColor pressed);
		ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor, ofColor idle, ofColor pressed,
					 float height, float width, float paddingH, float paddingV);
		ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor, ofImage* idle, ofImage* pressed);
		ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor, ofImage* idle, ofImage* pressed,
					 float height, float width, float paddingH, float paddingV);
		virtual void draw();		

		inline int getObjectId() { return obj->getId(); }
		void updateObject(const IObjectPtr& object);
	protected:
		virtual bool isInTouch(const DataTouch& touch);
		Polygon3D polygon;
	private:
		void init();
		void calculateFloorPolygon();
		void calculateFacePolygon();

		bool checkInFullObject(const DataTouch& touch);
		bool checkInFace(const DataTouch& touch);
		bool checkInFloor(const DataTouch& touch);
		
		void drawInFullObject();
		void drawInFace();
		void drawInFloor();
		void drawFace(const IPolygonPtr& pol);
		void drawFace(const Polygon3D& pol);

		OButtonMode oMode;
		IObjectPtr obj;
		IPolygonName face;

		float height;
		float width;
		float paddingH;
		float paddingV;
		bool calculatedPolygon;
	};
}

#endif //OBJECT_BUTTON_H__