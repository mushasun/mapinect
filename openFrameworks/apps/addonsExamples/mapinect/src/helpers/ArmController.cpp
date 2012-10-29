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

	ofVec3f ArmController::lookAt(const ofVec3f& target)
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
		return arduino->setArm3dCoordinates(position, true);
	}

	void ArmController::stopFollowing()
	{
		arduino->stopFollowing();
	}

	ofVec3f	ArmController::moveMotor(int motorId, signed int degrees)
	{
		return arduino->moveMotor(motorId, degrees);
	}

	ofVec3f	ArmController::setArmPositionAndLookAt(const ofVec3f& position, const ofVec3f& target)
	{
		ofVec3f pos = arduino->setArm3dCoordinates(position,false);
		ofVec3f lookingAt = arduino->lookAt(target);
		if (lookingAt == NULL ) {
			cout << "lookingAt dio null" << endl;
			arduino->reset(true);
			return NULL;
		} 
	
		return pos;
	}

}
