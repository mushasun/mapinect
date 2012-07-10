#include "ObjectButton.h"
#include "pointutils.h"
#include "ofGraphicsUtils.h"

namespace mapinect {

	void ObjectButton::init()
	{
		if(oMode == kInFloor)
			calculateFloorPolygon();
		else if(oMode == kInFace && calculatedPolygon)
			calculateFacePolygon();

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

		vertexs.push_back(v1);
		vertexs.push_back(v2);
		vertexs.push_back(v2 + norm * height);
		vertexs.push_back(v1 + norm * height);

		polygon = Polygon3D(vertexs);
	}
	
	//--------------------------------------------------------------
	void ObjectButton::calculateFacePolygon()
	{
		vector<ofVec3f> vertexs;
		IPolygonPtr pol = obj->getPolygon(face);
		ofVec3f v1 = pol->getMathModel().getVertexs().at(1);
		ofVec3f v2 = pol->getMathModel().getVertexs().at(2);
		ofVec3f norm = (pol->getMathModel().getVertexs().at(0) - v1).normalized();
		ofVec3f normV1V2 = (v2 - v1).normalized();

		v1 = v1 + normV1V2 * paddingH;
		v1 = v1 + norm * paddingV;

		v2 = v1 + normV1V2 * width;

		vertexs.push_back(v1);
		vertexs.push_back(v2);
		vertexs.push_back(v2 + norm * height);
		vertexs.push_back(v1 + norm * height);

		polygon = Polygon3D(vertexs);
	}

	//--------------------------------------------------------------
	void ObjectButton::updateObject(const IObjectPtr& object) 
	{ 
		obj = object; 
		if(oMode == kInFloor)
			calculateFloorPolygon();
		else if(oMode == kInFace && calculatedPolygon)
			calculateFacePolygon();
	}

	//--------------------------------------------------------------
	ObjectButton::ObjectButton(const IObjectPtr& obj, ofColor idle, ofColor pressed):
	BaseButton(idle,pressed), obj(obj)
	{
		calculatedPolygon = false;
		oMode = kFullObject;
		init();
	}

	//--------------------------------------------------------------
	ObjectButton::ObjectButton(const IObjectPtr& obj, ofImage* idle, ofImage* pressed):
	BaseButton(idle,pressed), obj(obj)
	{
		calculatedPolygon = false;
		oMode = kFullObject;
		init();
	}

	//--------------------------------------------------------------
	ObjectButton::ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor, ofColor idle, ofColor pressed):
	BaseButton(idle,pressed), obj(obj), face(face)
	{
		calculatedPolygon = false;
		oMode = mapToFloor ? kInFloor : kInFace;
		init();
	}

	//--------------------------------------------------------------
	ObjectButton::ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor, ofImage* idle, ofImage* pressed):
	BaseButton(idle,pressed), obj(obj), face(face)
	{
		calculatedPolygon = false;
		oMode = mapToFloor ? kInFloor : kInFace;
		init();
	}

	//--------------------------------------------------------------
	ObjectButton::ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor, ofColor idle, ofColor pressed,
							   float height, float width, float paddingH, float paddingV):
	BaseButton(idle,pressed), obj(obj), face(face), height(height), width(width), paddingH(paddingH), paddingV(paddingV)
	{
		calculatedPolygon = true;
		oMode = mapToFloor ? kInFloor : kInFace;
		init();
	}

	//--------------------------------------------------------------
	ObjectButton::ObjectButton(const IObjectPtr& obj, IPolygonName face, bool mapToFloor, ofImage* idle, ofImage* pressed,
							   float height, float width, float paddingH, float paddingV):
	BaseButton(idle,pressed), obj(obj), face(face), height(height), width(width), paddingH(paddingH), paddingV(paddingV)
	{
		calculatedPolygon = true;
		oMode = mapToFloor ? kInFloor : kInFace;
		init();
	}

	//--------------------------------------------------------------
	bool ObjectButton::isInTouch(const DataTouch& touch)
	{
		switch(oMode)
		{
			case kFullObject:
				return checkInFullObject(touch);
			case kInFace:
				return checkInFace(touch);
			case kInFloor:
				return checkInFloor(touch);
			default:
				return checkInFullObject(touch);
		}
	}

	//--------------------------------------------------------------
	void ObjectButton::draw()
	{
		switch(oMode)
		{
			case kFullObject:
				return drawInFullObject();
			case kInFace:
				return drawInFace();
			case kInFloor:
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
		for (vector<IPolygonPtr>::const_iterator p = obj->getPolygons().begin(); p != obj->getPolygons().end(); ++p)
		{
			if((*p)->getName() == face)
				drawFace(*p);
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
		if(mode == kBgImg)
		{
			ofSetColor(255,255,255,255);
			ofImage* tex = isPressed() ? texPressed : texIdle;
			tex->bind();
			ofDrawQuadTextured(pol.getVertexs(), ofTexCoordsFor(*tex));
			tex->unbind();
		}
		else if(mode == kColor)
		{
			if(isPressed())
				ofSetColor(pressedColor);
			else
				ofSetColor(idleColor);
			
			vector<ofVec3f> vertexs = pol.getVertexs();
			glBegin(GL_QUADS);      
				glVertex3f(vertexs.at(0).x, vertexs.at(0).y, vertexs.at(0).z); 
				glVertex3f(vertexs.at(1).x, vertexs.at(1).y, vertexs.at(1).z);
				glVertex3f(vertexs.at(2).x, vertexs.at(2).y, vertexs.at(2).z);
				glVertex3f(vertexs.at(3).x, vertexs.at(3).y, vertexs.at(3).z);
			glEnd();
		}
	}

	//--------------------------------------------------------------
	bool ObjectButton::checkInFullObject(const DataTouch& touch)
	{
		bool touched = false;
		for (vector<IPolygonPtr>::const_iterator p = obj->getPolygons().begin(); p != obj->getPolygons().end() && !touched; ++p)
			touched = (*p)->getMathModel().isInPolygon(touch.getTouchPoint());

		return touched;
	}
	
	//--------------------------------------------------------------
	bool ObjectButton::checkInFace(const DataTouch& touch)
	{
		return face == touch.getPolygon()->getName();
	}

	//--------------------------------------------------------------
	bool ObjectButton::checkInFloor(const DataTouch& touch)
	{
		return polygon.isInPolygon(touch.getTouchPoint());
	}
}