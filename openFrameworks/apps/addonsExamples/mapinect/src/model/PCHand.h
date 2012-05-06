#ifndef MAPINECT_PC_HAND_H__
#define MAPINECT_PC_HAND_H__

#include "PCModelObject.h"

namespace mapinect {

	class PCHand : public PCModelObject {
		public:
			PCHand(const PCPtr& cloud, int objId);
			
			virtual void		draw();
			virtual void		detectPrimitives();
			virtual void		applyTransformation();
			virtual void		resetLod();
			virtual void		increaseLod();

			inline const vector<ofVec3f>& getFingerTips() { return fingerTips; }

		private:
			virtual void		unifyVertexs();
			vector<ofVec3f>		fingerTips;
	};
}

#endif	// MAPINECT_PC_HAND_H__