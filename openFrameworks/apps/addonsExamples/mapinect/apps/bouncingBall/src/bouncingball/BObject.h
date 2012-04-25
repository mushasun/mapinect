#ifndef BOBJECT_H__
#define BOBJECT_H__

#include "ofVec3f.h"
#include "Segment3D.h"
#include "PCPolygon.h"
#include "PCPolyhedron.h"

using namespace mapinect;

namespace bouncing {
	class BObject {
		public:
			BObject(){ this->polyhedron = NULL; }
			BObject(std::vector<Segment3D> segments, ofVec3f color, int id, int soundId);
			void draw();
			void update();
			inline void clearSegments(){ segments.clear(); }
			inline void pushSegment(Segment3D& s){ segments.push_back(s); }
			inline int getId(){ return id; }
			inline void setPolyhedron(PCPolyhedron* hedron){ polyhedron = hedron; }
			inline std::vector<Segment3D> getSegments(){ return segments; }
			bool visited;
			ofSoundPlayer sound;
		
			inline int getColorBoost(){ return colorBoost; }
			void setColorBoost(int boost);
		private:
			std::vector<Segment3D> segments;
			ofVec3f color;
			int id;
			PCPolyhedron*	polyhedron;
			int colorBoost;
			float lastTime;
			//string sound;

		};
}

#endif	// BOBJECT_H__
