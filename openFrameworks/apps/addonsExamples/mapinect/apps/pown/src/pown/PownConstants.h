#ifndef POWNCONSTANTS_H__
#define POWNCONSTANTS_H__

namespace pown
{
	class PownConstants
	{
	public:

		static float				EMIT_TIME;

		static float				SPOT_BASE_RADIUS;
		static float				SPOT_ROTATION_PERIOD_TIME;

		static float				BOLT_INITIAL_SPEED;
		static float				BOLT_BASE_RADIUS;
		static float				BOLT_INTENSITY_DECREASE_FACTOR;
		static float				BOLT_BOOST_COLOR_DECREASE_FACTOR;

		static void					LoadPownConstants();

	};
}

#endif	// POWNCONSTANTS_H__
