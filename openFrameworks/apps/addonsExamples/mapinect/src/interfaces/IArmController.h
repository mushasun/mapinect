#ifndef I_ARM_CONTROLLER_H__
#define I_ARM_CONTROLLER_H__

#include "ofVec3f.h"
#include "IPolygon.h"

/// <summary>
/// Interface for arm controller. Entry point to handle the robotic arm
/// by moving its joints manually or automatic for camera positioning
/// </summary>

namespace mapinect {

	class IArmController {
		
	public:

		virtual bool			isReachable(const ofVec3f& eye, ofVec3f& bestFit) = 0;

		/// <summary>
		/// Arm will try to position the camera at 'eye' looking to 'target'
		/// Returns true if the 'eye' is reachable, else sets 'bestFit' with the closer position
		/// that will reach the arm and position the camera there
		/// </summary>
		virtual ofVec3f			lookAt(const ofVec3f& eye, const ofVec3f& target, ofVec3f& bestFit) = 0;

		virtual ofVec3f			getEye() = 0;
		virtual ofVec3f			getTarget() = 0;
		ofVec3f					getKinect3dCoordinates();

		/// <summary>
		/// Arm will try to rotate the joint an 'amount' of degrees.
		/// Returns true if the 'amount' is completely rotated, else sets 'bestFit' with closer value
		/// that will reach the joint and rotates there
		/// </summary>
		virtual int				getJointRotation(int jointId) = 0;

		/// <summary>
		/// The arm will try to always lookAt the object
		/// </summary>
		virtual void			followObject(const IObjectPtr& ) = 0;
		virtual void			stopFollowing() = 0;
		virtual ofVec3f			setArm3dCoordinates(const ofVec3f& ) = 0;

	};
}

#endif	// I_ARM_CONTROLLER_H__
