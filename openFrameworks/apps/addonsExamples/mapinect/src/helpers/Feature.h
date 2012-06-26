#ifndef FEATURE_H__
#define FEATURE_H__

namespace mapinect
{
	enum Feature
	{
		FEATURE_ARDUINO = 0,
		FEATURE_KINECT,
		FEATURE_CV,
		FEATURE_PCM,
		FEATURE_VM,
		FEATURE_SAVE_CLOUD,
		FEATURE_RECTANGLE_VERTEX,
		FEATURE_SHOW_RGB,
		FEATURE_HAND_DETECTION,
		FEATURE_MOVE_ARM,
		FEATURE_UNIFORM_DENSITY,

		FEATURE_COUNT
	};

	void LoadFeatures();

	bool IsFeatureActive(Feature feature);

	bool IsFeatureArduinoActive();
	bool IsFeatureKinectActive();
	bool IsFeatureCVActive();
	bool IsFeaturePCMActive();
	bool IsFeatureVMActive();
	bool IsFeatureSaveCloudActive();
	bool IsFeatureMoveArmActive();
	bool IsFeatureUniformDensity();

	#define CHECK_ACTIVE		if (!isActive()) return
}

#endif	// FEATURE_H__