#ifndef SOUND_MANAGER_H__
#define SOUND_MANAGER_H__

#include "ofxMidi.h"

namespace pown
{
	class SoundManager
	{
	public:
		static void				setup();

		static void				update(float elapsedTime);
		static void				playNote(int note);
		static void				setProgram(int program);
		static void				setBeatPeriod(float period);

	private:
		SoundManager();

		void					myUpdate(float elapsedTime);
		void					myPlayNote(int note);
		void					mySetProgram(int program);
		void					mySetBeatPeriod(float period);

		static SoundManager*	instance;

		ofxMidiOut				midiOut;
		int						channel;
		int						program;
		float					beatPeriod;

		float					timer;
	};
}

#endif	// SOUND_MANAGER_H__
