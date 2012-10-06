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
		FEATURE_ICP,
		FEATURE_UNIFORM_DENSITY,
		FEATURE_CALIBRATE_TABLE,
		FEATURE_INVALIDATE_OBJECT_BY_VOLUME,
		FEATURE_ENABLE_MAPPING_WHILE_MOVING,
		FEATURE_LOG,

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
	bool IsFeatureICPActive();
	bool IsFeatureUniformDensityActive();
	bool IsFeatureCalibrateTableActive();
	bool IsFeatureEnableMappingWhileMovingActive();
	bool IsFeatureLogActive();

	#define CHECK_ACTIVE		if (!isActive()) return
}

#endif	// FEATURE_H__