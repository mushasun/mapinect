#ifndef UTILS_H__
#define UTILS_H__

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