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
			gFeatures[FEATURE_ICP] = XML.getValue(FEATURE_CONFIG "ICP", false);
			gFeatures[FEATURE_RECTANGLE_VERTEX] = XML.getValue(FEATURE_CONFIG "RECTANGLE_VERTEX", false);
			gFeatures[FEATURE_SHOW_RGB] = XML.getValue(FEATURE_CONFIG "SHOW_RGB", false);
			gFeatures[FEATURE_TOUCH_DETECTION] = XML.getValue(FEATURE_CONFIG "TOUCH_DETECTION", true);
			gFeatures[FEATURE_OBJECT_DETECTION] = XML.getValue(FEATURE_CONFIG "OBJECT_DETECTION", true);
			gFeatures[FEATURE_MOVE_ARM] = XML.getValue(FEATURE_CONFIG "MOVE_ARM",false);
			gFeatures[FEATURE_UNIFORM_DENSITY] = XML.getValue(FEATURE_CONFIG "UNIFORM_DENSITY", true);
			gFeatures[FEATURE_CALIBRATE_TABLE] = XML.getValue(FEATURE_CONFIG "CALIBRATE_TABLE", false);
			gFeatures[FEATURE_INVALIDATE_OBJECT_BY_VOLUME] = XML.getValue(FEATURE_CONFIG "INVALIDATE_OBJECT_BY_VOLUME", true);
			gFeatures[FEATURE_ENABLE_MAPPING_WHILE_MOVING] = XML.getValue(FEATURE_CONFIG "FEATURE_ENABLE_MAPPING_WHILE_MOVING", true);
			gFeatures[FEATURE_LOG] = XML.getValue(FEATURE_CONFIG "FEATURE_LOG", false);
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

	bool IsFeatureMoveArmActive()
	{
		return IsFeatureActive(FEATURE_MOVE_ARM);
	}

	bool IsFeatureICPActive()
	{
		return IsFeatureActive(FEATURE_ICP);
	}

	bool IsFeatureUniformDensityActive()
	{
		return IsFeatureActive(FEATURE_UNIFORM_DENSITY);
	}

	bool IsFeatureCalibrateTableActive() 
	{
		return IsFeatureActive(FEATURE_CALIBRATE_TABLE);
	}

	bool IsFeatureEnableMappingWhileMovingActive() 
	{
		return IsFeatureActive(FEATURE_ENABLE_MAPPING_WHILE_MOVING);
	}

	bool IsFeatureLogActive() 
	{
		return IsFeatureActive(FEATURE_LOG);
	}

}
