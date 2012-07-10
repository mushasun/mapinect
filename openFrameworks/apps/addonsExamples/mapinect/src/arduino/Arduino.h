#ifndef ARDUINO_H__
#define ARDUINO_H__

#include <math.h>
#include <map>
#include <Eigen/Geometry>

#include "INotification.h"
#include "IObject.h"
#include "ofSerial.h"
#include "ofVec3f.h"
#include "pointUtils.h"
#include "EventManager.h"

namespace mapinect
{
	class Arduino : public INotification
	{
	
	public:
		virtual ~Arduino();
		Arduino();

		virtual bool		setup();
		virtual void		exit();
		virtual void		update();
		virtual void		draw();

		virtual void		keyPressed(int key);

		void				reset();

		ofVec3f				getKinect3dCoordinates();
		ofVec3f				setArm3dCoordinates(const ofVec3f& position);
		ofVec3f				lookAt(const ofVec3f& point);
		ofVec3f				lookingAt();
		Eigen::Affine3f		getWorldTransformation();
		Eigen::Affine3f		calculateWorldTransformation(float angle1, float angle2, float angle4, float angle8);
		void				objectUpdated(const IObjectPtr&);
		void				followObject(const IObjectPtr&);
		
		inline bool			isArmMoving() {	return armMoving;	}

		char*				read();
		
		signed int*			motorAngles() const;

		static float		ARM_LENGTH;
		static float		KINECT_HEIGHT;
		static float		MOTORS_HEIGHT;

	private:
		bool				isActive();
		void				sendMotor(int value, int id);
		ofVec3f				bestFitForArmSphere(const ofVec3f& point);
		void				setArm3dCoordinates(float x, float y, float z);

		ofSerial			serial;
		signed int			angleMotor1;
		signed int			angleMotor2;
		signed int			angleMotor4;
		signed int			angleMotor8;
		ofVec3f				posicion;
		ofVec3f				mira;
		ofVec3f				miraActual;
		bool				armStoppedMoving;
		bool				armMoving;
		static Eigen::Affine3f		worldTransformation;
		PCPtr				cloudBeforeMoving;
		PCPtr				cloudAfterMoving;
		int					id_object_to_follow;
		ofVec3f				center_of_following_object;
		void				applyICPLoadedClouds(); 

	};
}

#endif
