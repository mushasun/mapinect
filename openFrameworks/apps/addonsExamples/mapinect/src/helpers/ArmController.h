#ifndef ARM_CONTROLLER_H__
#define ARM_CONTROLLER_H__

#include "IArmController.h"
#include "INotification.h"
#include "IObject.h"

#include "Arduino.h"
#include <map>

namespace mapinect {

	class ArmController : public IArmController, public INotification
	{
	public:
		ArmController(Arduino* f);

		//IArmController Methods
		bool			isReachable(const ofVec3f& eye, ofVec3f& bestFit);
		bool			lookAt(const ofVec3f& eye, const ofVec3f& target, ofVec3f& bestFit);
		inline ofVec3f	getEye()		{ return eye; }
		inline ofVec3f	getTarget()		{ return target; }
		bool			rotateJoint(int jointId, int amount, int& bestFit);
		int				getJointRotation(int jointId);
		void			lookAtObject(const IObjectPtr&);

		//IObjectNotification methods
		void objectDetected(const IObjectPtr&);
		void objectUpdated(const IObjectPtr&);
		void objectLost(const IObjectPtr&);
		void objectMoved(const IObjectPtr&, const DataMovement&);
		void objectTouched(const IObjectPtr&, const DataTouch&);

	private:
		map<int, int>	jointIdMapping;
		Arduino*		arduino;
		ofVec3f			eye;
		ofVec3f			target;
		IObjectPtr		object_to_follow;

	};
}

#endif	// ARM_CONTROLLER_H__
