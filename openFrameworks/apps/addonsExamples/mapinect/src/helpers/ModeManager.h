#ifndef MODE_MANAGER_H__
#define MODE_MANAGER_H__

#include "IModeManager.h"
#include "PCM.h"

namespace mapinect
{
	class ModeManager: public IModeManager
	{
	public:
		inline ModeManager(PCM* pcm) { this->pcm = pcm; }

		//// IModeManager implementation
		virtual void inline enableObjectTracking()		{ pcm->objectDetectionEnabled(true); }
		virtual void inline disableObjectTracking()		{ pcm->objectDetectionEnabled(false); }
		virtual void inline enableTouchTracking()		{ pcm->touchDetectionEnabled(true); }
		virtual void inline disableTouchTracking()		{ pcm->touchDetectionEnabled(false); }
		virtual bool inline objectTrackingEnabled()		{ return pcm->isObjectDetectionEnabled(); }
		virtual bool inline touchTrackingEnabled()		{ return pcm->isTouchDetectionEnabled(); }
	private:
		PCM* pcm;

	};
}

#endif	// BUTTON_MANAGER_H__
