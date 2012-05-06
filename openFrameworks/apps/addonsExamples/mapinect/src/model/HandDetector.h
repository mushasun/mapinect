#ifndef MAPINECT_HANDDETECTOR_H__
#define MAPINECT_HANDDETECTOR_H__

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "PCPolygon.h"

using namespace pcl;

namespace mapinect {
	class HandDetector {
	public:
		void				SetPotentialHandCloud(const PCPtr& cloud);
		void				SetTable(Table* table);
		float				IsHand();
		int					GetHandDirection();

	private:
		void				trimPointsOutsideTable();
		void				trimHand();
		vector<ofVec3f>		getFingers();
		vector<ofVec3f>		unifyHandVertex(const PCPtr& handHull);
		float				checkFingers(vector<ofVec3f> fingers);
		int					findCloserFingerTo(ofVec3f currentFinger,vector<ofVec3f> unifiedHull,int handDirection);

		PCPtr				hand;
		Table*				table;
		bool				isHand;
		int					handDirection;
		PointXYZ			tipPoint;
		PointXYZ			handCentroid;
	};
}

#endif	// MAPINECT_HANDDETECTOR_H__