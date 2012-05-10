#ifndef MAPINECT_HANDDETECTOR_H__
#define MAPINECT_HANDDETECTOR_H__

#include "mapinectTypes.h"

namespace mapinect {
	class HandDetector {
	public:
		void				SetPotentialHandCloud(const PCPtr& cloud);
		void				SetTable(const TablePtr& table);
		float				IsHand();
		int					GetHandDirection();

	private:
		void				trimPointsOutsideTable();
		void				trimHand();
		vector<ofVec3f>		getFingers();
		vector<ofVec3f>		unifyHandVertex(const PCPtr& handHull);
		float				checkFingers(vector<ofVec3f>& fingers);
		int					findCloserFingerTo(const ofVec3f& currentFinger, const vector<ofVec3f>& unifiedHull, int handDirection);

		PCPtr				hand;
		TablePtr			table;
		bool				isHand;
		int					handDirection;
		pcl::PointXYZ		tipPoint;
		pcl::PointXYZ		handCentroid;
	};
}

#endif	// MAPINECT_HANDDETECTOR_H__