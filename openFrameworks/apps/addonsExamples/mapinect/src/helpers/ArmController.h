#ifndef ARM_CONTROLLER_H__
#define ARM_CONTROLLER_H__

#include "IArmController.h"

#include "Arduino.h"
#include <map>

namespace mapinect {

	class ArmController : public IArmController
	{
	public:
		ArmController(Arduino* f);

		bool			isReachable(const ofVec3f& eye, ofVec3f& bestFit);
		bool			lookAt(const ofVec3f& eye, const ofVec3f& target, ofVec3f& bestFit);

		inline ofVec3f	getEye()		{ return eye; }
		inline ofVec3f	getTarget()		{ return target; }

		bool			rotateJoint(int jointId, int amount, int& bestFit);

		int				getJointRotation(int jointId);

	private:
		map<int, int>	jointIdMapping;
		Arduino*		arduino;
		ofVec3f			eye;
		ofVec3f			target;

	};
}

#endif	// ARM_CONTROLLER_H__
