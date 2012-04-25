#ifndef MAPINECT_PC_HAND_H__
#define MAPINECT_PC_HAND_H__

#include "PCModelObject.h"

namespace mapinect {

	class PCHand : public PCModelObject {
		public:
			PCHand(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ>::Ptr extendedCloud, int objId);
			
			virtual void		draw();
			virtual void		detectPrimitives();
			virtual void		applyTransformation();
			inline list<ofVec3f>		getFingerTips() { return fingerTips; }
			virtual void		resetLod();
			virtual void		increaseLod();
		protected:
			//virtual PCPolygon*	createPCPolygon();

		private:
			virtual void		unifyVertexs();
			list<ofVec3f>		fingerTips;
	};
}

#endif	// MAPINECT_PC_HAND_H__