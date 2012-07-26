#ifndef BOLT_H__
#define BOLT_H__

#include "ofVec3f.h"
#include "ofColor.h"

namespace pown {

	class Bolt {
	public:
		Bolt(const ofColor& color, const ofVec3f& initialPosition, const ofVec3f& initialSpeed);
		virtual ~Bolt();

		inline float	getIntensity() const		{ return intensity; }
		inline ofColor	getColor() const			{ return color; }
		inline ofVec3f	getPosition() const			{ return position; }

		void			update(float elapsedTime);
		void			draw();

		bool			isAlive() const;
		float			radius() const;
		void			absorb();

	private:
		float			intensity;
		ofColor			color;
		ofVec3f			position;
		ofVec3f			speed;
		
	};
}

#endif	// BOLT_H__
