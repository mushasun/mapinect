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

#include "ICPThread.h"

#define		COULD_NOT_REACH		1
#define		CANT_MOVE			1

#define		ID_MOTOR_1			1
#define		ID_MOTOR_2			2
#define		ID_MOTOR_4			4
#define		ID_MOTOR_8			8

namespace mapinect
{
	class ICPThread;

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
		ofVec3f				setArm3dCoordinates(const ofVec3f& position, bool setArmStartedMoving);
		ofVec3f				moveMotor(int motorId, signed int degrees);
		ofVec3f				lookAt(const ofVec3f& point);
		ofVec3f				lookingAt();
		Eigen::Affine3f		calculateWorldTransformation(float angle1, float angle2, float angle4, float angle8);
		void				objectUpdated(const IObjectPtr&);
		void				followObject(const IObjectPtr&);
		void				stopFollowing();
		
		char*				read();
		
		signed int*			motorAngles() const;

		static float		ARM_HEIGHT;
		static float		ARM_LENGTH;
		static float		MOTORS_HEIGHT;
		static float		MOTORS_WIDTH;
		static float		KINECT_MOTOR_HEIGHT;
		static float		TILT_ANGLE;
		static float		KINECT_HEIGHT;

		inline bool			isArmMoving()	{	return startedMoving;	}

	private:
		bool				isActive();
		void				sendMotor(int value, int id);
		ofVec3f				bestFitForArmSphere(const ofVec3f& point);
		void				setArm3dCoordinates(float x, float y, float z, bool setArmStartedMoving);

		ofSerial			serial;
		signed int			angleMotor1;
		signed int			angleMotor2;
		signed int			angleMotor4;
		signed int			angleMotor8;
		ofVec3f				posicion;
		ofVec3f				mira;
		ofVec3f				miraActual;
		bool				startedMoving;
		bool				stoppedMoving;
		bool				isMoving;
		PCPtr				cloudBeforeMoving;
		PCPtr				cloudAfterMoving;
		int					idObjectToFollow;
		ofVec3f				centerOfFollowingObject;
		ofVec3f				acceleration;

		void				armStartedMoving();
		void				armStoppedMoving();

		ICPThread*			icpThread;

		void				loadXMLSettings();

	};
}

#endif
