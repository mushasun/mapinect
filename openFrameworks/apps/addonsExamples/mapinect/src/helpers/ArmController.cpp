#include "ArmController.h"

namespace mapinect {

	ArmController::ArmController(Arduino* arduino)
		: arduino(arduino), eye(0, 0, 0), target(0, 0, 1)
	{
		// initialize joint id mapping, should be dynamic
	}

	bool ArmController::isReachable(const ofVec3f& eye, ofVec3f& bestFit)
	{
		return true;
	}

	ofVec3f ArmController::lookAt(const ofVec3f& eye, const ofVec3f& target, ofVec3f& bestFit)
	{
		return arduino->lookAt(target);
	}

	int ArmController::getJointRotation(int jointId)
	{
		return 0;
	}

	void ArmController::followObject(const IObjectPtr& object)
	{
		arduino->followObject(object);
	}

	ofVec3f	ArmController::getKinect3dCoordinates()
	{
		return arduino->getKinect3dCoordinates();
	}

	ofVec3f	ArmController::setArm3dCoordinates(const ofVec3f& position)
	{
		return arduino->setArm3dCoordinates(position);
	}

	void ArmController::stopFollowing()
	{
		arduino->stopFollowing();
	}

	ofVec3f	ArmController::moveMotor(int motor_id, signed int degrees)
	{
		return arduino->moveMotor(motor_id, degrees);
	}
}
