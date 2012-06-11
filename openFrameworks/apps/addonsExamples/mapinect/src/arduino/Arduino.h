#ifndef ARDUINO_H__
#define ARDUINO_H__

#include <math.h>
#include <map> 
#include "ofSerial.h"
#include "ofVec3f.h"
#include <Eigen/Geometry>

namespace mapinect {
	class Arduino{
	
	public:
		Arduino();
		virtual ~Arduino();

		virtual bool		setup();
		virtual void		exit();
		virtual void		update();
		virtual void		draw();

		virtual void		keyPressed(int key);

		void				reset();

		ofVec3f				getKinect3dCoordinates();
		ofVec3f				setKinect3dCoordinates(ofVec3f position);
		ofVec3f				lookAt(ofVec3f point);
		ofVec3f				lookingAt();
		Eigen::Affine3f		getWorldTransformation();
		char*				read();
		

	private:
		bool				isActive();

		void				sendMotor(int value, int id);
		


		ofSerial			serial;
		signed int			angleMotor1;
		signed int			angleMotor2;
		signed int			angleMotor4;
		signed int			angleMotor8;
		ofVec3f				posicion;
		ofVec3f				mira;
		
		float round(float input)
		{
			//VC++ no tiene un puto round que lo tengo que hacer a mano???
			return floor(input < 0 ? input - 0.5 : input + 0.5);
		}


		ofVec3f				convert_3D_cart_to_spher(ofVec3f point);
		ofVec3f				convert_3D_spher_to_cart(ofVec3f point);
		ofVec3f				find_closest_point_to_sphere(ofVec3f point);
		void				setKinect3dCoordinates(float x, float y, float z);

		void		getTransformationWorldTransformation();
		
	};
}

#endif
