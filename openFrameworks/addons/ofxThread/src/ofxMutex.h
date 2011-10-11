#ifndef OFXMUTEX_H__
#define OFXMUTEX_H__

#include "ofConstants.h"

#ifdef TARGET_WIN32
	#include <process.h>
	
	typedef CRITICAL_SECTION mutex_t;
#else
    #include <pthread.h>

	typedef pthread_mutex_t mutex_t;
#endif

class ofxMutex {
	public:
		ofxMutex(bool verbose = false);
		virtual		~ofxMutex();
		
		void		lock();
		bool		tryLock();
		void		unlock();
		
	private:
		mutex_t		myMutex;
		bool		verbose;
};

#endif	// OFXMUTEX_H__
