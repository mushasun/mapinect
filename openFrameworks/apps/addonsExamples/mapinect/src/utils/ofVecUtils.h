#ifndef OFVEC_UTILS_H__
#define OFVEC_UTILS_H__

#include "ofVec3f.h"
#include "ofVec2f.h"
#include "ofPoint.h"

#include "utils.h"

extern ofVec3f BAD_OFVEC3F;
extern ofVec2f BAD_OFVEC2F;

string ofVecToString(const ofVec3f& v);
string ofVecToString(const ofVec3f& v, int precision);

void computeBoundingBox(const std::vector<ofVec3f>& v, ofVec3f& min, ofVec3f& max);

template<class T>
T computeNormal(const vector<T>& v)
{
	assert(v.size() > 2);
	return computeNormal(v[0], v[1], v[2]);
}

template<class T>
T computeNormal(const T& v1, const T& v2, const T& v3)
{
	return (v1 - v2).getCrossed(v3 - v2);
}

template<class T>
T computeCentroid(const vector<T>& v)
{
	T result;
	for (vector<T>::const_iterator it = v.begin(); it != v.end(); ++it)
	{
		result += *it;
	}
	result /= (float)(v.size());
	return result;
}

vector<vector<ofVec3f> > findClusters(const vector<ofVec3f>& v, float tolerance, float minClusterSize);

inline ofVec3f scaleFromMtsToMms(const ofVec3f& p) { return p * 1000; }

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

#define SORT_ON_PROP(T, method, prop, compare) \
	template<class T> \
	bool sortOn##method(const T& a, const T&b) { \
		return a.##prop ##compare b.##prop; \
	}

SORT_ON_PROP(T, XAsc, x, <)
SORT_ON_PROP(T, XDesc, x, >)
SORT_ON_PROP(T, YAsc, y, <)
SORT_ON_PROP(T, YDesc, y, >)
SORT_ON_PROP(T, ZAsc, z, <)
SORT_ON_PROP(T, ZDesc, z, >)
SORT_ON_PROP(T, DistAsc, dist, <)

#endif	// OFVEC_UTILS_H__