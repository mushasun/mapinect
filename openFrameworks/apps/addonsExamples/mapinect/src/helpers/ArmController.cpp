#include "ArmController.h"
#include "pointUtils.h"


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

	void ArmController::setWorldTransform()
	{
		// Set transformation matrix to apply to point cloud, in pointUtils::getPartialCloudRealCoords
		setTransformMatrix(arduino->getWorldTransformation());	
		// Set position and lookat for VM
		arduino->getKinect3dCoordinates();
	}

}
