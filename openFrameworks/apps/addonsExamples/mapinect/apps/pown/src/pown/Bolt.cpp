#include "Bolt.h"

#include "ofGraphicsUtils.h"
#include "ofLight.h"
#include "PownConstants.h"

#define INTENSITY_ALIVE_PERCENT		0.05f

namespace pown
{
	static ofLight boltLight;

	void Bolt::setup()
	{
		boltLight.setPointLight();
		boltLight.setAmbientColor(ofColor(kRGBWhite));
	}

	Bolt::Bolt(const ofColor& color, const ofVec3f& initialPosition, const ofVec3f& initialSpeed)
		: color(color), position(initialPosition), speed(initialSpeed)
	{
		this->color.a = 128;
		intensity = 1.0f;
	}

	Bolt::~Bolt()
	{
	}

	void Bolt::draw()
	{
		ofSetColor(color);
		ofPushMatrix();
		/*
			boltLight.enable();
			boltLight.setPosition(position);
			ofColor diffuseColor(color);
			diffuseColor.a = 64;
			boltLight.setDiffuseColor(diffuseColor);
			boltLight.setSpecularColor(diffuseColor);
			//ofRotateX(90);
			//ofEllipse(0, 0, 0, radius(), radius());
		*/
			ofCircle(position, radius());
		//	boltLight.disable();
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
