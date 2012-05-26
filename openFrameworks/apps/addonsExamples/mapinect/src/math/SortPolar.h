#ifndef MAPINECT_SORTPOLAR_H__
#define MAPINECT_SORTPOLAR_H__

#include "ofVec3f.h"
#include "ofVecUtils.h"
#include "ofPolar.h"

namespace mapinect
{
	struct SortPolar
	{
	public:
		SortPolar(const vector<ofVec3f>& vertexs)
			: vertexs(vertexs)
		{
			center = computeCentroid(vertexs);
			discardCoord = calculateDiscardCoordinate(vertexs);
		}

		virtual ~SortPolar() { };

		bool				operator()(const ofVec3f& a, const ofVec3f& b)
		{
			ofVec2f v2A = discardCoordinateOfVec3f(a - center, discardCoord);
			ofVec2f v2B = discardCoordinateOfVec3f(b - center, discardCoord);

			ofPolar pA(v2A);
			ofPolar pB(v2B);

			return pA.theta < pB.theta;
		}

	private:
		vector<ofVec3f>		vertexs;
		DiscardCoordinate	discardCoord;
		ofVec3f				center;

	};
}

#endif	// MAPINECT_SORTPOLAR_H__
