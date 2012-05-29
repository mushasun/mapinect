#include "Feature.h"

#include "ofxXmlSettings.h"

namespace mapinect
{
	static bool gFeatures[FEATURE_COUNT];

	#define		FEATURE_CONFIG			"FeatureConfig:FEATURE_"

	void LoadFeatures()
	{
		for (int i = 0; i < FEATURE_COUNT; i++)
		{
			gFeatures[i] = true;
		}

		ofxXmlSettings XML;
		if(XML.loadFile("Feature_Config.xml")) {
			gFeatures[FEATURE_ARDUINO] = XML.getValue(FEATURE_CONFIG "ARDUINO", true);
			gFeatures[FEATURE_KINECT] = XML.getValue(FEATURE_CONFIG "KINECT", true);
			gFeatures[FEATURE_CV] = XML.getValue(FEATURE_CONFIG "CV", true);
			gFeatures[FEATURE_PCM] = XML.getValue(FEATURE_CONFIG "PCM", true);
			gFeatures[FEATURE_VM] = XML.getValue(FEATURE_CONFIG "VM", true);
			gFeatures[FEATURE_SAVE_CLOUD] = XML.getValue(FEATURE_CONFIG "SAVE_CLOUD", false);
			gFeatures[FEATURE_RECTANGLE_VERTEX] = XML.getValue(FEATURE_CONFIG "RECTANGLE_VERTEX", false);

		}

	}

	bool IsFeatureActive(Feature feature)
	{
		return gFeatures[feature];
	}

	bool IsFeatureArduinoActive()
	{
		return IsFeatureActive(FEATURE_ARDUINO);
	}

	bool IsFeatureKinectActive()
	{
		return IsFeatureActive(FEATURE_KINECT);
	}

	bool IsFeatureCVActive()
	{																					 
		return IsFeatureActive(FEATURE_CV);
	}

	bool IsFeaturePCMActive()
	{
		return IsFeatureActive(FEATURE_PCM);
	}

	bool IsFeatureVMActive()
	{
		return IsFeatureActive(FEATURE_VM);
	}

	bool IsFeatureSaveCloudActive()
	{
		return IsFeatureActive(FEATURE_SAVE_CLOUD);
	}

}
