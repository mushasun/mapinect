#include "River.h"

#include "ofGraphicsUtils.h"
#include "DraggableButton.h"
#define NORMAL_FACTOR 5

namespace story
{
	River::River()
	{
	}

	River::River(const ofVec3f begin, const ofVec3f end)
		: begin(begin), end(end)
	{
		texture = new ofImage("data/texturas/road/road.jpg");
		normal = ofVec3f(0, 1, 0);
		ofVec3f direccion = normal.cross(begin-end).normalize() * NORMAL_FACTOR;
		vector<ofVec3f> draggable;
		draggable.push_back(begin + direccion);
		draggable.push_back(begin - direccion);
		draggable.push_back(end + direccion);
		draggable.push_back(end - direccion);
		Polygon3D area = Polygon3D(draggable);
		DraggableButton d1( draggable,
							texture,
							texture);
	}

	River::~River()
	{
	}

	void River::update(float elapsedTime)
	{
		//dibujar autos?
	}

	void River::draw()
	{
		//no va el dibujado porque lo hace el button
	}

}