#ifndef OFVEC_UTILS_H__
#define OFVEC_UTILS_H__

#include "ofVec3f.h"
#include "ofVec2f.h"
#include "ofPoint.h"

#include "utils.h"

void findOfxVec3fBoundingBox(const std::vector<ofVec3f>& v, ofVec3f &vMin, ofVec3f &vMax);


enum DiscardCoordinate {
	kDiscardCoordinateX,
	kDiscardCoordinateY,
	kDiscardCoordinateZ
};

DiscardCoordinate calculateDiscardCoordinate(const ofVec3f& v);
DiscardCoordinate calculateDiscardCoordinate(const ofVec3f& min, const ofVec3f& max);
ofVec2f discardCoordinateOfxVec3f(const ofVec3f& v, DiscardCoordinate discard);


struct ofPolar {
	float ro, theta;
};

inline ofVec3f scaleFromMtsToMms(const ofVec3f& p) { return p * 1000; }

ofPolar cartesianToPolar(const ofPoint& c);

bool sortOnY(const ofVec3f& l, const ofVec3f& r);

bool sortOnX(const ofVec3f& l, const ofVec3f& r);

bool sortOnZ(const ofVec3f& l, const ofVec3f& r);


template<class T>
int indexOf(const std::vector<T>& v, const T& t)
{
	for (int i = 0; i < v.size(); i++) {
		T vi(v[i]);
		if (vi == t) {
			return i;
		}
	}
	return -1;
}


struct PairMatching {
	int ixA, ixB;
};


template<class T>
void _permutate(const std::vector<T>& v, std::vector<std::vector<T>>& solution, std::vector<T>& p)
{
	if (p.size() == v.size()) {
		solution.push_back(p);
	}
	else {
		for (int i = 0; i < v.size(); i++) {
			if (indexOf(p, v[i]) < 0) {
				p.push_back(v[i]);
				_permutate(v, solution, p);
				p.pop_back();
			}
		}
	}
}

template<class T>
std::vector<std::vector<T>> permutations(const std::vector<T>& v)
{
	std::vector<std::vector<T>> result;
	std::vector<T> p;
	_permutate(v, result, p);
	return result;
}

template<class T>
std::vector<PairMatching> bestMatching(const std::vector<T>& vA, const std::vector<T>& vB, float(*evalFunc)(const T&, const T&))
{
	std::vector<PairMatching> result;

	std::vector<std::vector<T>>	perms = permutations(vB);

	int ixPerm = -1;
	if (true || perms.size() == 0) {
		perms.push_back(vB);
		ixPerm = 0;
	}
	else {
		float minSum = MAX_FLOAT;

		for (int i = 0; i < perms.size(); i++) {
			float sum = 0;
			for (int j = 0; j < perms[i].size(); j++) {
				sum += evalFunc(vA[j], perms[i][j]);
			}
			if (sum < minSum) {
				minSum = sum;
				ixPerm = i;
			}
		}

	}
	for (int i = 0; i < perms[ixPerm].size(); i++) {
		PairMatching pair;
		pair.ixA = i;
		pair.ixB = indexOf(vB, perms[ixPerm][i]);
		result.push_back(pair);
	}

	return result;
}

template<class T>
T average(const std::vector<T>& v) {
	T avg;
	for (int i = 0; i < v.size(); i++) {
		T vi(v[i]);
		avg += vi;
	}
	avg /= v.size();
	return avg;
}

#define SORT_ON_PROP(T, prop) \
	template<class T> \
	bool sortOn_##prop(const T& a, const T&b) { \
		return a.##prop < b.##prop; \
	}


SORT_ON_PROP(T, x)
SORT_ON_PROP(T, y)
SORT_ON_PROP(T, z)

#endif	// OFVEC_UTILS_H__