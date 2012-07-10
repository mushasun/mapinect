#include "Timer.h"

#include <iostream>

using namespace std;

namespace mapinect {

	Timer::Timer()
	{
		startTime = time(NULL);
		endTime = startTime;
	}

	void Timer::start() {
		startTime = time(NULL);
	}

	void Timer::end() {
		endTime = time(NULL);
	}

	void Timer::print() {
		cout << getElapsedSeconds() << " sec" << endl;
	}

	float Timer::getElapsedSeconds() {
		return (float)(endTime - startTime);
	}

}
