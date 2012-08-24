#ifndef OBJECT_BUTTON_H__
#define OBJECT_BUTTON_H__

#include "BaseButton.h"

#include "IObject.h"

namespace mapinect {
	enum ObjectButtonType
	{
		kObjectButtonTypeOnFace,
		kObjectButtonTypeOnTable,
		kObjectButtonTypeFullObject
	};

	class ObjectButton;
	typedef boost::shared_ptr<ObjectButton> ObjectButtonPtr;

	class ObjectButton : public BaseButton
	{
	public:
		ObjectButton(const IObjectPtr& obj, const ofColor& idle, const ofColor& pressed);
		ObjectButton(const IObjectPtr& obj, ofImage* idle, ofImage* pressed);
		ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor, const ofColor& idle, const ofColor& pressed);
		ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor, const ofColor& idle, const ofColor& pressed,
					 float height, float width, float paddingH, float paddingV);
		ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor, ofImage* idle, ofImage* pressed);
		ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor, ofImage* idle, ofImage* pressed,
					 float height, float width, float paddingH, float paddingV);
		
		virtual void draw();

		inline int getObjectId() { return obj->getId(); }
		void updateObject(const IObjectPtr& object);
		inline virtual vector<ofVec3f>		getVertexs() { return polygon.getVertexs(); }

	protected:
		virtual bool isInTouch(const IObjectPtr& object, const DataTouch& touch);
		Polygon3D polygon;

	private:
		void init();
		void calculateFloorPolygon();
		void calculateFacePolygon();

		bool checkInFullObject(const DataTouch& touch, const IObjectPtr& object);
		bool checkInFace(const DataTouch& touch, const IObjectPtr& object);
		bool checkInFloor(const DataTouch& touch);
		
		void drawInFullObject();
		void drawInFace();
		void drawInFloor();
		void drawFace(const IPolygonPtr& pol);
		void drawFace(const Polygon3D& pol);

		ObjectButtonType buttonType;
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