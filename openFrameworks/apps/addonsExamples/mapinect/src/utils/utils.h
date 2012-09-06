#ifndef UTILS_H__
#define UTILS_H__

#include <vector>
#include <string>
#include <assert.h>

#define MATH_EPSILON		0.005
#define MATH_EPSILON_2		0.01

#define MAX_FLOAT			FLT_MAX

template<typename T>
T round(T x, int decimals = 0)
{
	T r = x;
	int i = decimals;
	while (i-- > 0)
		r *= 10.0;
	r = floor(r + 0.5);
	while (decimals-- > 0)
		r /= 10.0;
	return r;
}

template<typename T>
bool inRange(T x, T a, T b) { return a <= x && x <= b; }

template<typename T>
T moveIntoRange(T x, T a, T b) { assert(a <= b); return max(min(x, b), a); }

std::vector<float> parseArray(const std::string& str);

#endif	// UTILS_H__