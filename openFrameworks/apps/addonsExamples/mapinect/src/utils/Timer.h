#ifndef MAPINECT_TIMER_H__
#define MAPINECT_TIMER_H__

namespace mapinect {
	class Timer {
	public:
		Timer();
		virtual ~Timer() {};

		void			start();
		void			stop();
		void			print();
		float			getElapsedSeconds();
		float			stopAndGetElapsedSeconds();
		float			stopResumeAndGetElapsedSeconds();

	private:
		unsigned long	startTime, endTime;
	};
}

#endif	// MAPINECT_TIMER_H__