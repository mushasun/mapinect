#ifndef ARDUINO_H__
#define ARDUINO_H__

#include <math.h>
#include <map> 
#include "ofSerial.h"
#include "ofVec3f.h"
#include <Eigen/Geometry>
#include "pointUtils.h"

namespace mapinect {
	class Arduino{
	
	public:
		static Arduino* getInstance();
		virtual ~Arduino();
		Arduino();

		virtual bool		setup();
		virtual void		exit();
		virtual void		update();
		virtual void		draw();

		virtual void		keyPressed(int key);

		void				reset();

		ofVec3f				getKinect3dCoordinates();
		ofVec3f				setArm3dCoordinates(ofVec3f position);
		ofVec3f				lookAt(ofVec3f point);
		ofVec3f				lookingAt();
		Eigen::Affine3f		getWorldTransformation();
		Eigen::Affine3f		calculateWorldTransformation(float angle1, float angle2, float angle4, float angle8);
		
		inline bool			isArmMoving() {	return armMoving;	}

		char*				read();
		
		signed int*			motorAngles() const;

		static float		ARM_LENGTH;
		static float		KINECT_HEIGHT;
		static float		MOTORS_HEIGHT;

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
				
		ofVec3f				mira_actual;

		bool				armStoppedMoving;
		bool				armMoving;
		static Eigen::Affine3f		worldTransformation;
		PCPtr				cloudBeforeMoving;
		PCPtr				cloudAfterMoving;

		float round(float input)
		{
			//VC++ no tiene un puto round que lo tengo que hacer a mano???
			return floor(input < 0 ? input - 0.5 : input + 0.5);
		}


		ofVec3f				convert_3D_cart_to_spher(ofVec3f point);
		ofVec3f				convert_3D_spher_to_cart(ofVec3f point);
		ofVec3f				find_closest_point_to_sphere(ofVec3f point);
		void				setArm3dCoordinates(float x, float y, float z);

	};
}

#endif
