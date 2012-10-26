#include "SoundManager.h"

#include "PownConstants.h"
#include "utils.h"

#include <set>

namespace pown
{
	SoundManager* SoundManager::instance = NULL;

	const int VELOCITY = 64;
	const int BASE_PITCH = 57;
	const int BASE_SCALE_NOTES = 5;
	static int baseScale[BASE_SCALE_NOTES] = { 0, 2, 5, 7, 9 };
	static int* scale;
	const int programs[PROGRAMS] = { 3, 8, 9, 46, 51, 62, 88, 91, 96, 97, 98, 118 };

	struct ProgramNote
	{
		ProgramNote(int program, int note) : program(program), note(note) { }

		inline bool operator==(const ProgramNote& other) const
		{
			return program == other.program && note == other.note;
		}
		inline bool operator<(const ProgramNote& other) const
		{
			return program < other.program || (program == other.program && note < other.note);
		}

		int program;
		int note;
	};

	struct ProgramNotePlaying
	{
		ProgramNotePlaying(const ProgramNote& pn, int lifetime) : programNote(pn), lifetime(lifetime) { }

		inline bool isAlive() const { return lifetime > 0; }
		ProgramNote programNote;
		int lifetime;
	};

	static set<ProgramNote> notesToPlay;
	static list<ProgramNotePlaying> notesPlaying;

	const bool playSynchronized = true;

	SoundManager::SoundManager()
		: channel(1), program(0)
	{
		midiOut.openPort(0);
	}

	void SoundManager::setup()
	{
		scale = new int[PownConstants::NOTES];
		for (int i = 0; i < PownConstants::NOTES; i++)
			scale[i] = BASE_PITCH + baseScale[i % BASE_SCALE_NOTES] + 12 * (i / BASE_SCALE_NOTES);
		instance = new SoundManager();
	}

	void SoundManager::beat()
	{
		instance->myBeat();
	}

	void SoundManager::playNote(int note, int program)
	{
		instance->myPlayNote(note, program);
	}

	void SoundManager::setProgram(int program)
	{
		instance->mySetProgram(program);
	}

	int SoundManager::getProgram()
	{
		return instance->program;
	}

	void SoundManager::myBeat()
	{
		for (int i = 0; i < PownConstants::NOTES; i++)
			midiOut.sendNoteOff(channel, scale[i], VELOCITY);
		if (playSynchronized)
		{
			for (set<ProgramNote>::const_iterator pn = notesToPlay.begin(); pn != notesToPlay.end(); pn++)
				myPlayNoteNow(pn->note, pn->program);
			notesToPlay.clear();
		}
	}

	void SoundManager::myPlayNote(int note, int program)
	{
		if (!inRange(program, 0, PROGRAMS - 1))
			program = this->program;

		if (playSynchronized)
			notesToPlay.insert(ProgramNote(program, note));
		else
			myPlayNoteNow(note, program);
	}

	void SoundManager::myPlayNoteNow(int note, int program)
	{
		int saveProgram = this->program;
		mySetProgram(programs[3]);
		midiOut.sendNoteOn(channel, scale[note], VELOCITY);
		mySetProgram(saveProgram);
	}

	void SoundManager::mySetProgram(int program)
	{
		this->program = program;
		midiOut.sendProgramChange(channel, program);
	}

}
