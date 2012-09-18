#include "ofVecUtils.h"

#include "utils.h"
#include "ofUtils.h"

ofVec3f BAD_OFVEC3F(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
ofVec2f BAD_OFVEC2F(MAX_FLOAT, MAX_FLOAT);

string ofVecToString(const ofVec3f& v)
{
	const int kDefaultPrecision = 2;
	return ofVecToString(v, kDefaultPrecision);
}

string ofVecToString(const ofVec3f& v, int precision)
{
	return "(" + ofToString(v.x, precision) + ", " +
		ofToString(v.y, precision) + ", " +
		ofToString(v.z, precision) + ")";
}

void computeBoundingBox(const std::vector<ofVec3f>& v, ofVec3f& vMin, ofVec3f& vMax) {
	vMin = ofVec3f(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
	vMax = ofVec3f(-MAX_FLOAT, -MAX_FLOAT, -MAX_FLOAT);

	for (int k = 0; k < v.size(); k++) {
		ofVec3f p = v.at(k);
		vMin.x = min(p.x, vMin.x);
		vMin.y = min(p.y, vMin.y);
		vMin.z = min(p.z, vMin.z);
		vMax.x = max(p.x, vMax.x);
		vMax.y = max(p.y, vMax.y);
		vMax.z = max(p.z, vMax.z);
	}

}

vector<vector<ofVec3f> > findClusters(const vector<ofVec3f>& vecs, float tolerance, float minClusterSize)
{
	vector<ofVec3f> r(vecs);

	vector<vector<ofVec3f> > clusters;
	while (r.begin() != r.end())
	{
		ofVec3f v(*r.begin());
		vector<vector<ofVec3f> >::iterator closerCluster = clusters.end();
		float minDist = tolerance;
		for (vector<vector<ofVec3f> >::iterator cluster = clusters.begin(); cluster != clusters.end(); ++cluster)
		{
			for (vector<ofVec3f>::const_iterator i = cluster->begin(); i != cluster->end(); ++i)
			{
				float dist = v.distance(*i);
				if (dist < minDist)
				{
					minDist = dist;
					closerCluster = cluster;
				}
			}
		}
		if (closerCluster == clusters.end())
		{
			closerCluster = clusters.insert(clusters.begin(), vector<ofVec3f>());
		}
		closerCluster->push_back(v);
		r.erase(r.begin());
	}

	vector<vector<ofVec3f> > result;
	for (vector<vector<ofVec3f> >::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
		if (i->size() > minClusterSize)
			result.push_back(vector<ofVec3f>(i->begin(), i->end()));

	return result;
}
