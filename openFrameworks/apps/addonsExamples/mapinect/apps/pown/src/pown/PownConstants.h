#ifndef POWNCONSTANTS_H__
#define POWNCONSTANTS_H__

namespace pown
{
	class PownConstants
	{
	public:

		static int					NOTES;
		static int					BEATS;

		static float				EMIT_TIME;
		static float				BEAT_TIME;

		static float				SPOT_PERIOD_TIME;
		static float				SPOT_SEED_TIME;

		static float				BOX_BEAT_TIME;

		static float				WAVE_INTENSITY_DECREASE_FACTOR;
		static float				WAVE_RADIUS_INCREASE_TIME;

		static void					LoadPownConstants();

	};
}

#endif	// POWNCONSTANTS_H__
