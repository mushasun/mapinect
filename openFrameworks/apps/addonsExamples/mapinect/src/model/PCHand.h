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
			
			vector<Polygon3D>	getMathModelApproximation() const { return vector<Polygon3D>(); }

			inline const vector<ofVec3f>& getFingerTips() { return fingerTips; }
			void				addToModel(const PCPtr& cloud);
		private:
			virtual void		unifyVertexs();
			vector<ofVec3f>		fingerTips;
	};
}

#endif	// MAPINECT_PC_HAND_H__