#include "Bolt.h"

#include "ofGraphics.h"
#include "PownConstants.h"

#define INTENSITY_ALIVE_PERCENT		0.05f

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
		ofPushMatrix();
			ofTranslate(position);
			ofRotateX(90);
			ofEllipse(0, 0, 0, radius(), radius());
		ofPopMatrix();
	}

	void Bolt::update(float elapsedTime)
	{
		position += speed * elapsedTime;
		intensity -= PownConstants::BOLT_INTENSITY_DECREASE_FACTOR * elapsedTime;
	}

	bool Bolt::isAlive() const
	{
		return intensity > INTENSITY_ALIVE_PERCENT;
	}

	float Bolt::radius() const
	{
		return PownConstants::BOLT_BASE_RADIUS * intensity;
	}

	void Bolt::absorb()
	{
		intensity = 0.0f;
	}

}
