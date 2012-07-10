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

	bool ArmController::lookAt(const ofVec3f& eye, const ofVec3f& target, ofVec3f& bestFit)
	{
		return true;
	}

	bool ArmController::rotateJoint(int jointId, int amount, int& bestFit)
	{
		return true;
	}

	int ArmController::getJointRotation(int jointId)
	{
		return 0;
	}

	void ArmController::lookAtObject(const IObjectPtr& object)
	{
		arduino->followObject(object);
	}

	void ArmController::objectDetected(const IObjectPtr& object)
	{
	}

	void ArmController::objectUpdated(const IObjectPtr& object)
	{
	}
	
	void ArmController::objectLost(const IObjectPtr& object)
	{
		if (object_to_follow != NULL)
		{
			if (object_to_follow->getId() == object->getId())
			{
				object_to_follow.reset();
			}
		}
	}

	void ArmController::objectMoved(const IObjectPtr& object, const DataMovement& data)
	{
		if (object_to_follow != NULL)
		{
			if (object_to_follow->getId() == object->getId())
			{
				ofVec3f old_center = object_to_follow->getCenter();
				ofVec3f new_center = object->getCenter();
				if (old_center.distance(new_center) > 0.02)
				{
					object_to_follow = object;
					arduino->lookAt(new_center);
				}
			}
		}
	}

	void ArmController::objectTouched(const IObjectPtr& object, const DataTouch& data)
	{
	}

}
