#ifndef UTILS_H__
#define UTILS_H__

#include <vector>
#include <string>

#define KINECT_DEFAULT_WIDTH		640
#define KINECT_DEFAULT_HEIGHT		480

extern int					objId;

#define MAX_FLOAT			FLT_MAX

inline float minf(float a, float b) { return a < b ? a : b; }
inline float maxf(float a, float b) { return a > b ? a : b; }

std::vector<float> parseArray(const std::string& str);

#endif	// UTILS_H__