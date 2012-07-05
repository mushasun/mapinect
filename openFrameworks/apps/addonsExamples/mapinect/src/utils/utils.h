#ifndef UTILS_H__
#define UTILS_H__

#include <vector>
#include <string>
#include <assert.h>

#define KINECT_DEFAULT_WIDTH		640
#define KINECT_DEFAULT_HEIGHT		480

#define MATH_EPSILON		0.005

#define MAX_FLOAT			FLT_MAX

template<typename T>
T round(T x) { return floor(x + 0.5); }

template<typename T>
bool inRange(T x, T a, T b) { return a <= x && x <= b; }

template<typename T>
T moveIntoRange(T x, T a, T b) { assert(a <= b); return max(min(x, b), a); }

std::vector<float> parseArray(const std::string& str);

#endif	// UTILS_H__