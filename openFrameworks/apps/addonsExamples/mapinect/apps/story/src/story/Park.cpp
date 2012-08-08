#include "Park.h"

#include "ofGraphicsUtils.h"
#include "DraggableButton.h"


namespace story
{
	Park::Park(const ofVec3f begin, const ofVec3f end)
		: begin(begin), end(end)
	{
		texture = new ofImage("img/road.jpg");
		normal = ofVec3f(0, 1, 0);
		ofVec3f direccion = normal.cross(begin-end).normalize() * (begin-end).length()/2;
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

	Park::~Park()
	{
	}

	void Park::update(float elapsedTime)
	{
		//dibujar autos?
	}

	void Park::draw()
	{

	}

}