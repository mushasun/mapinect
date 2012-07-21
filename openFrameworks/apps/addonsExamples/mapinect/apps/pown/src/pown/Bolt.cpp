#include "Bolt.h"

#include "ofGraphics.h"

#define INTENSITY_ALIVE_PERCENT		0.05f
#define INTENSITY_DECREASE_FACTOR	5.0f
#define BASE_SIZE					0.01f

namespace pown
{
	Bolt::Bolt(const ofColor& color, const ofVec3f& initialPosition, const ofVec3f& initialSpeed)
		: color(color), position(initialPosition), speed(initialSpeed)
	{
		intensity = 1.0f;
	}

	Bolt::~Bolt()
	{
	}

	void Bolt::draw()
	{
		ofSetColor(color);
		ofCircle(position.x, position.y, position.z, size());
	}

	void Bolt::update(float elapsedTime)
	{
		position += speed * elapsedTime;
		intensity -= INTENSITY_DECREASE_FACTOR * elapsedTime;
	}

	bool Bolt::isAlive() const
	{
		return intensity > INTENSITY_ALIVE_PERCENT;
	}

	float Bolt::size() const
	{
		return BASE_SIZE * intensity;
	}

	void Bolt::absorb()
	{
		intensity = 0.0f;
	}

}
