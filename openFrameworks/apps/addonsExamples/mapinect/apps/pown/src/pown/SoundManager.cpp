#include "SoundManager.h"

#include "PownConstants.h"

namespace pown
{
	SoundManager* SoundManager::instance = NULL;

	const int VELOCITY = 64;
	const int BASE_PITCH = 57;
	const int NOTES = 16;
	static int pentatonicScale[NOTES] = { 0, 2, 5, 7, 9, 12, 14, 17, 19, 21, 24, 26, 29, 31, 33, 36 };
	static bool notes[NOTES];

	const bool playSynchronized = true;

	SoundManager::SoundManager()
		: channel(1), program(0)
	{
		timer = 0;
		mySetBeatPeriod(PownConstants::EMIT_TIME * 0.25f);
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

	void SoundManager::playNote(int note)
	{
		instance->myPlayNote(note);
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
				for (int i = 0; i < NOTES; i++)
				{
					if (notes[i])
						midiOut.sendNoteOn(channel, BASE_PITCH + pentatonicScale[i], VELOCITY);
					notes[i] = false;
				}
			}
			timer -= beatPeriod;
		}
	}

	void SoundManager::myPlayNote(int note)
	{
		if (playSynchronized)
			notes[note] = true;
		else
			midiOut.sendNoteOn(channel, BASE_PITCH + pentatonicScale[note], VELOCITY);
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
