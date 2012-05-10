#ifndef BOBJECT_H__
#define BOBJECT_H__

#include "ofVec3f.h"
#include "Segment3D.h"
#include "PCPolygon.h"
#include "ofSoundPlayer.h"

using namespace mapinect;

namespace bouncing {
	class BObject {
		public:
			BObject(const vector<Segment3D>& segments, const ofVec3f& color, int id, int soundId);

			void draw();
			void update();
			inline void clearSegments()						{ segments.clear(); }
			inline void addSegment(const Segment3D& s)		{ segments.push_back(s); }
			inline int getId()								{ return id; }
			inline void setModelObject(PCPolygon* p)		{ modelObject = p; }
			inline const vector<Segment3D>& getSegments()	{ return segments; }
			bool visited;
			ofSoundPlayer sound;
		
			inline int getColorBoost()						{ return colorBoost; }
			void setColorBoost(int boost);

		private:
			vector<Segment3D>		segments;
			ofVec3f					color;
			int						id;
			PCPolygon*				modelObject;
			int						colorBoost;
			float					lastTime;
			//string				sound;

		};
}

#endif	// BOBJECT_H__
