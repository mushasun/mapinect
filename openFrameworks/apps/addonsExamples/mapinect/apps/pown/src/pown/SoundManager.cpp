#include "SoundManager.h"

#include "PownConstants.h"
#include "utils.h"

namespace pown
{
	SoundManager* SoundManager::instance = NULL;

	const int VELOCITY = 64;
	const int BASE_PITCH = 57;
	static int pentatonicScale[NOTES] = { 0, 2, 5, 7, 9, 12, 14, 17, 19, 21, 24, 26, 29, 31, 33, 36 };

	struct ProgramNote
	{
		ProgramNote(int program, int note) : program(program), note(note) { }
		int program;
		int note;
	};
	static vector<ProgramNote> notesToPlay;

	const bool playSynchronized = true;

	SoundManager::SoundManager()
		: channel(1), program(0)
	{
		timer = 0;
		mySetBeatPeriod(PownConstants::EMIT_TIME / (float)BEATS);
		midiOut.openPort(0);
	}

	void SoundManager::setup()
	{
		instance = new SoundManager();
	}

	void SoundManager::update(float elapsedTime)
	{
		instance->myUpdate(elapsedTime);
	}

	void SoundManager::playNote(int note, int program)
	{
		instance->myPlayNote(note, program);
	}

	void SoundManager::setProgram(int program)
	{
		instance->mySetProgram(program);
	}

	void SoundManager::setBeatPeriod(float period)
	{
		instance->mySetBeatPeriod(period);
	}

	void SoundManager::myUpdate(float elapsedTime)
	{
		timer += elapsedTime;
		if (timer >= beatPeriod)
		{
			for (int i = 0; i < NOTES; i++)
				midiOut.sendNoteOff(channel, BASE_PITCH + pentatonicScale[i], VELOCITY);
			if (playSynchronized)
			{
				for (int i = 0; i < notesToPlay.size(); i++)
					myPlayNoteNow(notesToPlay[i].note, notesToPlay[i].program);
				notesToPlay.clear();
			}
			timer -= beatPeriod;
		}
	}

	void SoundManager::myPlayNote(int note, int program)
	{
		if (!inRange(program, MIN_PROGRAM, MAX_PROGRAM))
			program = this->program;

		if (playSynchronized)
			notesToPlay.push_back(ProgramNote(program, note));
		else
			myPlayNoteNow(note, program);
	}

	void SoundManager::myPlayNoteNow(int note, int program)
	{
		int saveProgram = this->program;
		//mySetProgram(program);
		midiOut.sendNoteOn(channel, BASE_PITCH + pentatonicScale[note], VELOCITY);
		//mySetProgram(saveProgram);
	}

	void SoundManager::mySetProgram(int program)
	{
		this->program = program;
		midiOut.sendProgramChange(channel, program);
	}

	void SoundManager::mySetBeatPeriod(float period)
	{
		beatPeriod = period;
	}

}
