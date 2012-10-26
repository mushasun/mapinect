#ifndef NOTEBEAT_H__
#define NOTEBEAT_H__

namespace pown {

	class NoteBeat
	{
	public:
		NoteBeat(int note, int beat) : note(note), beat(beat) { }
		NoteBeat(const NoteBeat& other) : note(other.note), beat(other.beat) { }

		bool		inRangeNote() const;
		bool		inRangeBeat() const;
		inline bool	inRange() const		{ return inRangeNote() && inRangeBeat(); }

		inline bool	operator==(const NoteBeat& other) const	{ return note == other.note && beat == other.beat; }
		bool operator<(const NoteBeat& other) const;

		int			note;
		int			beat;
	};
}

#endif	// NOTEBEAT_H__
