#ifndef SOUND_MANAGER_H__
#define SOUND_MANAGER_H__

#include "ofxMidi.h"

namespace pown
{
	const int PROGRAMS = 12;

	class SoundManager
	{
	public:
		static void				setup();

		static void				beat();
		static void				playNote(int note, int program = -1);
		static void				setProgram(int program);
		static int				getProgram();

	private:
		SoundManager();

		void					myBeat();
		void					myPlayNote(int note, int program = -1);
		void					myPlayNoteNow(int note, int program = -1);
		void					mySetProgram(int program);

		static SoundManager*	instance;

		ofxMidiOut				midiOut;
		int						channel;
		int						program;
	};
}

#endif	// SOUND_MANAGER_H__
