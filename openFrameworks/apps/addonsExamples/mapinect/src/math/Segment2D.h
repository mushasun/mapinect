#ifndef MAPINECT_SEGMENT2D_H__
#define MAPINECT_SEGMENT2D_H__

#include "Line2D.h"

namespace mapinect
{
	class Segment2D : public Line2D
	{
	public:
		Segment2D(const ofVec2f& origin, const ofVec2f& destination);
		virtual ~Segment2D() { };

		inline const ofVec2f& getDestination() const { return pDestination; }

		bool isInSegment(const ofVec2f& p) const;
		float directionScale(const ofVec2f& p) const;

	private:
		ofVec2f pDestination;
	};
}

#endif	// MAPINECT_SEGMENT2D_H__