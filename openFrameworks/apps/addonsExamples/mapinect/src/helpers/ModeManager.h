#ifndef MODE_MANAGER_H__
#define MODE_MANAGER_H__

#include "IModeManager.h"
#include "PCM.h"

namespace mapinect
{
	class ModeManager: public IModeManager
	{
	public:
		inline ModeManager(PCM* pcm) { _pcm = pcm; }

		//// IModeManager implementation
		virtual void inline enableObjectTracking()		{ _pcm->objectDetectionEnabled(true); }
		virtual void inline disableObjectTracking()		{ _pcm->objectDetectionEnabled(false); }
		virtual void inline enableTouchTracking()		{ _pcm->touchDetectionEnabled(true); }
		virtual void inline disableTouchTracking()		{ _pcm->touchDetectionEnabled(false); }
		virtual bool inline objectTrackingEnabled()		{ return _pcm->isObjectDetectionEnabled(); }
		virtual bool inline touchTrackingEnabled()		{ return _pcm->isTouchDetectionEnabled(); }
	private:
		PCM* _pcm;

	};
}

#endif	// BUTTON_MANAGER_H__
