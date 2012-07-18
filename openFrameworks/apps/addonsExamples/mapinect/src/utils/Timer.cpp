#include "Timer.h"

#include "ofUtils.h"
#include <iostream>

using namespace std;

namespace mapinect {

	Timer::Timer()
	{
		startTime = ofGetSystemTime();
		endTime = startTime;
	}

	void Timer::start()
	{
		startTime = ofGetSystemTime();
	}

	void Timer::stop()
	{
		endTime = ofGetSystemTime();
	}

	void Timer::print()
	{
		cout << getElapsedSeconds() << " sec" << endl;
	}

	float Timer::getElapsedSeconds()
	{
		return (float)(endTime - startTime) / 1000.0f;
	}

	float Timer::stopAndGetElapsedSeconds()
	{
		stop();
		return getElapsedSeconds();
	}

	float Timer::stopResumeAndGetElapsedSeconds()
	{
		stop();
		float result = getElapsedSeconds();
		startTime = endTime;
		return result;
	}

}
