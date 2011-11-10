#ifndef BOBJECT_H__
#define BOBJECT_H__

#include "ofxVec3f.h"
#include "Segment3D.h"

class BObject {
	public:
		BObject(std::vector<Segment3D> segments, ofxVec3f color, int id);
		void draw();
		inline int getId() { return id; }
		inline void setPolyedron(PCPolyhedron pol) { polyedron = pol; }

	private:
		std::vector<Segment3D> segments;
		ofxVec3f color;
		int id;
		PCPolyhedron polyedron;
		
		//string sound;

	};

#endif	// MAPINECT_LINE2D_H__