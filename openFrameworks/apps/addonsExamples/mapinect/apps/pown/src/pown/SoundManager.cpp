#include "SoundManager.h"

#include "PownConstants.h"
#include "utils.h"

namespace pown
{
	SoundManager* SoundManager::instance = NULL;

	const int VELOCITY = 64;
	const int BASE_PITCH = 57;
	const int BASE_SCALE_NOTES = 5;
	static int baseScale[BASE_SCALE_NOTES] = { 0, 2, 5, 7, 9 };
	static int* scale;

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

	void SoundManager::myBeat()
	{
		for (int i = 0; i < PownConstants::NOTES; i++)
			midiOut.sendNoteOff(channel, scale[i], VELOCITY);
		if (playSynchronized)
		{
			for (int i = 0; i < notesToPlay.size(); i++)
				myPlayNoteNow(notesToPlay[i].note, notesToPlay[i].program);
			notesToPlay.clear();
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
		mySetProgram(program);
		midiOut.sendNoteOn(channel, scale[note], VELOCITY);
		mySetProgram(saveProgram);
	}

	void SoundManager::mySetProgram(int program)
	{
		this->program = program;
		midiOut.sendProgramChange(channel, program);
	}

}
