#ifndef ARDUINO_H__
#define ARDUINO_H__

#include <math.h>
#include <map> 
#include "ofSerial.h"
#include "ofxVec3f.h"

namespace mapinect {
	class Arduino{
	
	public:
		Arduino();
		virtual ~Arduino();

		virtual bool	setup();
		virtual void	exit();
		virtual void	update();
		virtual void	draw();

		virtual void	keyPressed(int key);

		void			reset();

		ofxVec3f		getKinect3dCoordinates();
		ofxVec3f		setKinect3dCoordinates(ofxVec3f position);
		

	private:
		void			sendMotor(char value, int id);
		string			read();


		ofSerial		serial;
		signed int		angleMotor1;
		signed int		angleMotor2;
		signed int		angleMotor4;
		signed int		angleMotor8;
		
		float round(float input)
		{
			//VC++ no tiene un puto round que lo tengo que hacer a mano???
			return floor(input < 0 ? input - 0.5 : input + 0.5);
		}


		ofxVec3f		convert_3D_cart_to_spher(ofxVec3f point);
		ofxVec3f		convert_3D_spher_to_cart(ofxVec3f point);
		ofxVec3f		find_closest_point_to_sphere(ofxVec3f point);
		void		setKinect3dCoordinates(float x, float y, float z);

	};
}

#endif
