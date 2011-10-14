#include "Timer.h"

#include <iostream>

using namespace std;

namespace mapinect {
	void Timer::start() {
		startTime = time(NULL);
	}

	void Timer::end() {
		endTime = time(NULL);
	}

	void Timer::print() {
		cout << getElapsedMiliseconds() << " ms" << endl;
	}

	float Timer::getElapsedMiliseconds() {
		return (float)(endTime - startTime) / 1000.0f;
	}
}
