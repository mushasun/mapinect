#include "PownConstants.h"

#include "ofxXmlSettings.h"

#define POWN_CONFIG			"PownConfig:"

namespace pown
{
	int					PownConstants::NOTES								= 16;
	int					PownConstants::BEATS								= 16;

	float				PownConstants::EMIT_TIME							= 2.0f;
	float				PownConstants::BEAT_TIME							= EMIT_TIME / (float)BEATS;

	float				PownConstants::SPOT_PERIOD_TIME						= 2.0f;
	float				PownConstants::SPOT_SEED_TIME						= 5.0f;

	float				PownConstants::BOX_BEAT_TIME						= 0.5f;

	float				PownConstants::WAVE_INTENSITY_DECREASE_FACTOR		= 0.5f;
	float				PownConstants::WAVE_RADIUS_INCREASE_TIME			= 0.1f;

	void PownConstants::LoadPownConstants()
	{
		ofxXmlSettings XML;
		if(XML.loadFile("Pown_Config.xml"))
		{
			NOTES							= XML.getValue(POWN_CONFIG "NOTES", NOTES);
			BEATS							= XML.getValue(POWN_CONFIG "BEATS", BEATS);
			EMIT_TIME						= XML.getValue(POWN_CONFIG "EMIT_TIME", EMIT_TIME);
			BEAT_TIME						= EMIT_TIME / (float)BEATS;
			SPOT_PERIOD_TIME				= XML.getValue(POWN_CONFIG "SPOT_PERIOD_TIME", SPOT_PERIOD_TIME);
			SPOT_SEED_TIME					= XML.getValue(POWN_CONFIG "SPOT_SEED_TIME", SPOT_SEED_TIME);
			WAVE_INTENSITY_DECREASE_FACTOR	= XML.getValue(POWN_CONFIG "WAVE_INTENSITY_DECREASE_FACTOR", WAVE_INTENSITY_DECREASE_FACTOR);
			WAVE_RADIUS_INCREASE_TIME		= XML.getValue(POWN_CONFIG "WAVE_RADIUS_INCREASE_TIME", WAVE_RADIUS_INCREASE_TIME);
			BOX_BEAT_TIME					= XML.getValue(POWN_CONFIG "BOX_BEAT_TIME", BOX_BEAT_TIME);
		}
	}

}