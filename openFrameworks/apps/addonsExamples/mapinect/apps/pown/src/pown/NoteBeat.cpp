#include "NoteBeat.h"

#include "PownConstants.h"
#include "utils.h"

namespace pown
{
	bool NoteBeat::inRangeNote() const
	{
		return ::inRange(note, 0, PownConstants::NOTES - 1);
	}

	bool NoteBeat::inRangeBeat() const
	{
		return ::inRange(beat, 0, PownConstants::BEATS - 1);
	}

	bool NoteBeat::operator<(const NoteBeat& other) const
	{
		return note < other.note || (note == other.note && beat < other.beat);
	}
}
