#include "ofxThread.h" 

//------------------------------------------------- 
ofxThread::ofxThread() {
   threadRunning = false; 
   verbose = false;
} 

//------------------------------------------------- 
ofxThread::~ofxThread(){ 
   stopThread(); 
} 

//------------------------------------------------- 
bool ofxThread::isThreadRunning(){ 
   //should be thread safe - no writing of vars here 
   return threadRunning; 
} 

//------------------------------------------------- 
void ofxThread::startThread(bool _blocking, bool _verbose){ 
	if (threadRunning) {
		if(verbose)
			printf("ofxThread: thread already running\n"); 
		return; 
	} 

	//have to put this here because the thread can be running 
	//before the call to create it returns 
	threadRunning   = true; 

#ifdef TARGET_WIN32 
	//InitializeCriticalSection(&critSec); 
	myThread = (HANDLE)_beginthreadex(NULL, 0, this->thread,  (void *)this, 0, NULL); 
#else 
	//pthread_mutex_init(&myMutex, NULL); 
	pthread_create(&myThread, NULL, thread, (void *)this); 
#endif 

	blocking =   _blocking; 
	verbose = _verbose; 
}

//------------------------------------------------- 
bool ofxThread::lock() {
	if (blocking) {
		myMutex.lock();
		return true;
	}
	else {
		return myMutex.tryLock();
	}
}

//------------------------------------------------- 
void ofxThread::unlock() {
	myMutex.unlock();
}

//------------------------------------------------- 
void ofxThread::stopThread(bool close){
	if(threadRunning) {
		if (close) {
			#ifdef TARGET_WIN32
				CloseHandle(myThread);
			#else
				//pthread_mutex_destroy(&myMutex);
				pthread_detach(myThread);
			#endif
		}
		if (verbose)
			printf("ofxThread: thread stopped\n");
		threadRunning = false;
	}
	else {
		if (verbose)
			printf("ofxThread: thread already stopped\n");
	}
}

//-------------------------------------------------
void ofxThread::waitForThread(bool stop){
	if (threadRunning) {
		// Reset the thread state
		if(stop) {
			threadRunning = false;
			if(verbose)
				printf("ofxThread: stopping thread\n");
		}
		if(verbose)
			printf("ofxThread: waiting for thread to stop\n");
		// Wait for the thread to finish
		#ifdef TARGET_WIN32
			WaitForSingleObject(myThread, INFINITE);
			CloseHandle(myThread);
		#else
			if(pthread_self() == myThread)
				printf("ofxThread: error, waitForThread should only be called from outside the thread");
		    pthread_join(myThread, NULL);
		#endif
		if(verbose)
			printf("ofxThread: thread stopped\n");
		myThread = NULL;
	}
	else {
		if(verbose)
			printf("ofxThread: thread already stopped\n");
	}
}
