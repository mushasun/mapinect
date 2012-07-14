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

		void						applyICP(const PCPtr& cloudBefore, const PCPtr& cloudAfter);

		void						processICP();

	private:
		bool						checkApplyICP;
		PCPtr						cloudBeforeMoving;
		PCPtr						cloudAfterMoving;

		ofxMutex					icpMutex;	
	};
}

#endif	// ICP_THREAD_H__