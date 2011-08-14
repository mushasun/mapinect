#ifndef UTILS_H__
#define UTILS_H__

#define KINECT_WIDTH 640
#define KINECT_HEIGHT 480

#define MAX_FLOAT		FLT_MAX

inline double dabsd(double value) {
	if (value < 0) {
		return -value;
	}
	else {
		return value;
	}
}

#endif	// UTILS_H__