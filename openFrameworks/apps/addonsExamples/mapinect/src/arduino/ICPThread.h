#ifndef ICP_THREAD_H__
#define ICP_THREAD_H__

#include "mapinectTypes.h"
#include "ofThread.h"
#include "ofxMutex.h"

namespace mapinect
{
	class ICPThread : ofThread
	{
	public:
		ICPThread();

		void						reset();
		void						setup();
		void						exit();
		virtual void				threadedFunction();

		void						applyICP(const PCPtr& cloudBefore, const PCPtr& cloudAfter, int maxIterations);

		void						processICP();

	private:
		bool						checkApplyICP;
		PCPtr						cloudBeforeMoving;
		PCPtr						cloudAfterMoving;
		int							maxIterations;

		ofxMutex					icpMutex;	

		bool						goodTableEstimation(PCPtr newCloud, float maxAngleThreshold);
		bool						goodTableEstimationRedetectingTable(PCPtr nubeAfterMovingTransfICP);
	};
}

#endif	// ICP_THREAD_H__