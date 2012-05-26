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
		FEATURE_DEBUG_CLOUDS,

		FEATURE_COUNT
	};

	void LoadFeatures();

	bool IsFeatureActive(Feature feature);

	bool IsFeatureArduinoActive();
	bool IsFeatureKinectActive();
	bool IsFeatureCVActive();
	bool IsFeaturePCMActive();
	bool IsFeatureVMActive();

	#define CHECK_ACTIVE		if (!isActive()) return
}

#endif	// FEATURE_H__