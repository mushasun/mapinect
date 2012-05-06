#ifndef MAPINECT_POTENTIAL_HAND_H__
#define MAPINECT_POTENTIAL_HAND_H__

#include "mapinectTypes.h"
#include "ofVec3f.h"

namespace mapinect {
	class PotentialHand {
	public:
		PotentialHand(const PCPtr& cloud, const ofVec3f& centroid);
		PotentialHand(const PCPtr& cloud);

		PCPtr			cloud;
		bool			visited;
		int				timesVisited;
		ofVec3f			centroid;
		
		inline bool operator==(const PotentialHand &otherPotentialHand)
		{
			return (otherPotentialHand.centroid - this->centroid).length() < 0.005;
		}
	};
}

#endif	// MAPINECT_POTENTIAL_HAND_H__