#ifndef MAPINECT_TIMER_H__
#define MAPINECT_TIMER_H__

#include <time.h>

namespace mapinect {
	class Timer {
	public:
		Timer() {};
		virtual ~Timer() {};

		void start();
		void end();
		void print();
		float getElapsedMiliseconds();

	private:
		time_t	startTime, endTime;
	};
}

#endif	// MAPINECT_TIMER_H__