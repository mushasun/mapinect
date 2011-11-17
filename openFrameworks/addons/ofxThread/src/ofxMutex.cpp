#include "ofxMutex.h"

ofxMutex::ofxMutex(bool verbose) {
	this->verbose = verbose;
#ifdef TARGET_WIN32 
	InitializeCriticalSection(&myMutex); 
#else 
	pthread_mutex_init(&myMutex, NULL); 
#endif 
}

//------------------------------------------------- 
ofxMutex::~ofxMutex() {
#ifndef TARGET_WIN32 
	pthread_mutex_destroy(&myMutex); 
#endif 
}

//------------------------------------------------- 
void ofxMutex::lock(){
#ifdef TARGET_WIN32 
	EnterCriticalSection(&myMutex);
#else 
	if(verbose) {
		printf("ofxMutex: waiting till mutext is unlocked\n");
	}
	pthread_mutex_lock(&myMutex);
#endif 
	if(verbose) {
		printf("ofxMutex: we are in -- mutext is now locked \n");
	}
} 

//------------------------------------------------- 
bool ofxMutex::tryLock(){
#ifdef TARGET_WIN32 
	if(!TryEnterCriticalSection(&myMutex)){ 
		if(verbose) {
			printf("ofxMutex: mutext is busy \n"); 
		}
		return false; 
	}
#else 
	int value = pthread_mutex_trylock(&myMutex);
	if (value != 0) {
		if(verbose) {
			printf("ofxMutex: mutext is busy - already locked\n");
		}
		return false;
	}
#endif 
	if(verbose) {
		printf("ofxMutex: we are in -- mutext is now locked \n"); 
	}
	return true; 
} 

//------------------------------------------------- 
void ofxMutex::unlock(){ 
#ifdef TARGET_WIN32 
	LeaveCriticalSection(&myMutex); 
#else 
	pthread_mutex_unlock(&myMutex); 
#endif 
	
	if (verbose) {
		printf("ofxMutex: we are out -- mutex is now unlocked \n"); 
	}
} 

ofxScopedMutex::ofxScopedMutex(const ofxMutex& m) {
	mutex = m;
	mutex.lock();
}

ofxScopedMutex::~ofxScopedMutex() {
	mutex.unlock();
}
