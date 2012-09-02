#include "Road.h"

#include "ofGraphicsUtils.h"
#include "DraggableButton.h"
#include "DraggableButton.h"
#define NORMAL_FACTOR 0.08

namespace story
{
	Road::Road(const ofVec3f begin, const ofVec3f end, const Polygon3D& table)
		: begin(begin), end(end)
	{
		texture = new ofImage("data/texturas/road/road.jpg");
		normal = table.getPlane().getNormal();
		ofVec3f direccion = normal.cross(begin-end).normalize() * NORMAL_FACTOR;
		vector<ofVec3f> draggable;
		draggable.push_back(table.project(end + direccion));
		draggable.push_back(table.project(begin + direccion));
		draggable.push_back(table.project(begin - direccion));
		draggable.push_back(table.project(end - direccion));
		
		Polygon3D area = Polygon3D(draggable);
		button = IButtonPtr(new DraggableButton ( draggable,
							texture,
							texture));
	}

	Road::~Road()
	{
	}

	void Road::update(float elapsedTime)
	{
		//dibujar autos?
	}

	void Road::draw()
	{
		//no va el dibujado porque lo hace el button
	}

}