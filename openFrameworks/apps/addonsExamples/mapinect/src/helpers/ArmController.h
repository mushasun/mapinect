#ifndef ARM_CONTROLLER_H__
#define ARM_CONTROLLER_H__

#include "IArmController.h"
#include "INotification.h"
#include "IObject.h"

#include "Arduino.h"
#include <map>

namespace mapinect {

	class ArmController : public IArmController
	{
	public:
		ArmController(Arduino* f);

		//IArmController Methods
		bool			isReachable(const ofVec3f& eye, ofVec3f& bestFit);
		ofVec3f			lookAt(const ofVec3f& target);
		inline ofVec3f	getEye()		{ return eye; }
		inline ofVec3f	getTarget()		{ return target; }
		int				getJointRotation(int jointId);
		void			followObject(const IObjectPtr&);
		ofVec3f			setArm3dCoordinates(const ofVec3f& position);
		ofVec3f			getKinect3dCoordinates();
		ofVec3f			moveMotor(int motorId, signed int degrees);
		void			stopFollowing();

	private:
		map<int, int>	jointIdMapping;
		Arduino*		arduino;
		ofVec3f			eye;
		ofVec3f			target;
		IObjectPtr		objectToFollow;

	};
}

#endif	// ARM_CONTROLLER_H__
