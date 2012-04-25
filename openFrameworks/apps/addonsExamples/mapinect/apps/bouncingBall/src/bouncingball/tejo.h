#ifndef TEJO_H__
#define TEJO_H__

#include "ofVecUtils.h"
#include "ofMain.h"
#include "Segment3D.h"
#include "BObject.h"

namespace bouncing {
	class Tejo
	{
	public:

	
		Tejo() { }
		Tejo(ofVec3f position, double radio, ofVec3f direction, double vel, ofVec3f tableNorm, ofVec3f tableCenter);
		virtual ~Tejo() { }

		inline double getRadio() { return radio; }
		inline ofVec3f getPos() { return position; }

		void draw();
		void update(vector<BObject*> bobjects);

	private:
			ofVec3f position;
			ofVec3f direction;
			double vel;
			double radio;
			void move();
			bool check_collision(vector<BObject*> bobjects);
			//bool hayInterseccionEsfera(vector<Esfera*> cuerpos);
			static float AbsorcionEnergeticaEnChoque ;
			int idBalaTextura;
			float moduloVectorVelocidad;

			ofVec3f tableNormal;
			ofVec3f tableCenter;
	};
}

#endif	// TEJO_H__