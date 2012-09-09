#include "ObjectButton.h"
#include "ofGraphicsUtils.h"
#include "pointUtils.h"
#include "ofMain.h"
#include "Globals.h"

namespace mapinect {

	void ObjectButton::init()
	{
		if(buttonType == kObjectButtonTypeOnTable)
			calculateFloorPolygon();
		else if(buttonType == kObjectButtonTypeOnFace && calculatedPolygon)
			calculateFacePolygon();

	}

	//--------------------------------------------------------------
	ObjectButton::ObjectButton(const IObjectPtr& obj, const ofColor& idle, const ofColor& pressed)
		: BaseButton(idle, pressed), obj(obj)
	{
		calculatedPolygon = false;
		buttonType = kObjectButtonTypeFullObject;
		init();
	}

	//--------------------------------------------------------------
	ObjectButton::ObjectButton(const IObjectPtr& obj, ofImage* idle, ofImage* pressed)
		: BaseButton(idle, pressed), obj(obj)
	{
		calculatedPolygon = false;
		buttonType = kObjectButtonTypeFullObject;
		init();
	}

	//--------------------------------------------------------------
	ObjectButton::ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor,
		const ofColor& idle, const ofColor& pressed)
		: BaseButton(idle, pressed), obj(obj), face(face)
	{
		calculatedPolygon = false;
		buttonType = mapToFloor ? kObjectButtonTypeOnTable : kObjectButtonTypeOnFace;
		init();
	}

	//--------------------------------------------------------------
	ObjectButton::ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor, ofImage* idle, ofImage* pressed):
	BaseButton(idle, pressed), obj(obj), face(face)
	{
		calculatedPolygon = false;
		buttonType = mapToFloor ? kObjectButtonTypeOnTable : kObjectButtonTypeOnFace;
		init();
	}

	//--------------------------------------------------------------
	ObjectButton::ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor,
								const ofColor& idle, const ofColor& pressed,
								float height, float width, float paddingH, float paddingV)
		: BaseButton(idle,pressed), obj(obj), face(face), height(height), width(width), paddingH(paddingH), paddingV(paddingV)
	{
		calculatedPolygon = true;
		buttonType = mapToFloor ? kObjectButtonTypeOnTable : kObjectButtonTypeOnFace;
		init();
	}

	//--------------------------------------------------------------
	ObjectButton::ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor, ofImage* idle, ofImage* pressed,
							   float height, float width, float paddingH, float paddingV)
		: BaseButton(idle,pressed), obj(obj), face(face), height(height), width(width), paddingH(paddingH), paddingV(paddingV)
	{
		calculatedPolygon = true;
		buttonType = mapToFloor ? kObjectButtonTypeOnTable : kObjectButtonTypeOnFace;
		init();
	}

	//--------------------------------------------------------------
	void ObjectButton::calculateFloorPolygon()
	{
		vector<ofVec3f> vertexs;
		IPolygonPtr pol = obj->getPolygon(face);
		ofVec3f v1 = pol->getMathModel().getVertexs().at(1);
		ofVec3f v2 = pol->getMathModel().getVertexs().at(2);
		ofVec3f norm = pol->getMathModel().getPlane().getNormal();
		ofVec3f normV1V2 = (v2 - v1).normalized();

		v1 = v1 + normV1V2 * paddingH;
		v1 = v1 + norm * paddingV;

		v2 = v1 + normV1V2 * width;

		
		

		gModel->tableMutex.lock();
		TablePtr table = gModel->getTable();
		Plane3D plane (table->getCoefficients());
		gModel->tableMutex.unlock();

		vertexs.push_back(plane.project(v2));
		vertexs.push_back(plane.project(v2 + norm * height));
		vertexs.push_back(plane.project(v1 + norm * height));
		vertexs.push_back(plane.project(v1));

		polygon = Polygon3D(vertexs);
	}
	
	//--------------------------------------------------------------
	void ObjectButton::calculateFacePolygon()
	{
		vector<ofVec3f> vertexs;
		IPolygonPtr pol = obj->getPolygon(face);
		Polygon3D poly = obj->getPolygon(face)->getMathModel();
		ofVec3f v1 = pol->getMathModel().getVertexs().at(1);
		ofVec3f v2 = pol->getMathModel().getVertexs().at(2);
		ofVec3f faceNorm = pol->getMathModel().getPlane().getNormal();
		ofVec3f norm = (pol->getMathModel().getVertexs().at(0) - v1).normalized();
		ofVec3f normV1V2 = (v2 - v1).normalized();

		v1 = v1 + normV1V2 * paddingH;
		v1 = v1 + norm * paddingV;

		v2 = v1 + normV1V2 * width;

		ofVec3f zModifier = faceNorm * ZINDEX_MULT;

		vertexs.push_back(v1 + norm * height + zModifier);
		vertexs.push_back(v1 + zModifier);
		vertexs.push_back(v2 + zModifier);
		vertexs.push_back((v2 + norm * height) + zModifier);
		
		this->polygon = Polygon3D(vertexs);
	}

	//--------------------------------------------------------------
	void ObjectButton::updateObject(const IObjectPtr& object) 
	{ 
		obj = object; 
		if(buttonType == kObjectButtonTypeOnTable)
			calculateFloorPolygon();
		else if(buttonType == kObjectButtonTypeOnFace && calculatedPolygon)
			calculateFacePolygon();
	}

	//--------------------------------------------------------------
	bool ObjectButton::isInTouch(const IObjectPtr& object, const DataTouch& touch)
	{
			switch(buttonType)
			{
				case kObjectButtonTypeFullObject:
					return checkInFullObject(touch,object);
				case kObjectButtonTypeOnFace:
					return checkInFace(touch,object);
				case kObjectButtonTypeOnTable:
					return checkInFloor(touch);
				default:
					return checkInFullObject(touch, object);
			}
	
	}

	//--------------------------------------------------------------
	void ObjectButton::draw()
	{
		switch(buttonType)
		{
			case kObjectButtonTypeFullObject:
				return drawInFullObject();
			case kObjectButtonTypeOnFace:
 				return drawInFace();
			case kObjectButtonTypeOnTable:
				return drawInFloor();
			default:
				return drawInFullObject();
		}
	}

	//--------------------------------------------------------------
	void ObjectButton::drawInFullObject()
	{
		for (vector<IPolygonPtr>::const_iterator p = obj->getPolygons().begin(); p != obj->getPolygons().end(); ++p)
		{
			drawFace(*p);
		}
	}

	//--------------------------------------------------------------
	void ObjectButton::drawInFace()
	{
		if(calculatedPolygon)
			drawFace(polygon);
		else
		{
			for (vector<IPolygonPtr>::const_iterator p = obj->getPolygons().begin(); p != obj->getPolygons().end(); ++p)
			{
				if((*p)->getName() == face)
					drawFace(*p);
			}
		}
	}

	//--------------------------------------------------------------
	void ObjectButton::drawInFloor()
	{
		drawFace(polygon);
	}

	//--------------------------------------------------------------
	void ObjectButton::drawFace(const IPolygonPtr& pol)
	{
		drawFace(pol->getMathModel());
	}

	//--------------------------------------------------------------
	void ObjectButton::drawFace(const Polygon3D& pol)
	{
		if (mode == kButtonDrawModeTextured)
		{
			ofSetColor(kRGBWhite);
			ofImage* tex = isPressed() ? pressedTexture : idleTexture;
			tex->bind();
			ofDrawQuadTextured(pol.getVertexs(), ofTexCoordsFor());
			tex->unbind();
		}
		else if (mode == kButtonDrawModePlain)
		{
			if(isPressed())
				ofSetColor(pressedColor);
			else
				ofSetColor(idleColor);
			
			vector<ofVec3f> vertexs = pol.getVertexs();
			ofDrawQuad(vertexs);
		}
	}

	//--------------------------------------------------------------
	bool ObjectButton::checkInFullObject(const DataTouch& touch, const IObjectPtr& object)
	{
		if(object->getId() == obj->getId())
		{
	
			bool touched = false;
			for (vector<IPolygonPtr>::const_iterator p = obj->getPolygons().begin(); p != obj->getPolygons().end() && !touched; ++p)
				touched = (*p)->getMathModel().isInPolygon(touch.getTouchPoint());

			return touched;
		}
		else
			return false;
	}
	
	//--------------------------------------------------------------
	bool ObjectButton::checkInFace(const DataTouch& touch, const IObjectPtr& object)
	{
	
		if(object->getId() == obj->getId())
		{
			bool sameFace = face == touch.getPolygon()->getName();

			if(sameFace)
			{
				if(calculatedPolygon)
				{
					return polygon.isInPolygon(touch.getTouchPoint());
				}
				else
				{
					return true;
				}
			}
		}
		return false;
	}

	//--------------------------------------------------------------
	bool ObjectButton::checkInFloor(const DataTouch& touch)
	{
		saveCloud("touchPoint.pcd", touch.getTouchPoint());
		saveCloud("button.pcd", polygon.getVertexs());
		//cout << "in pol: " << polygon.isInPolygon(touch.getTouchPoint()) << endl;
		return polygon.isInPolygon(touch.getTouchPoint());
	}
}
