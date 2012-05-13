#ifndef GRAB_H__
#define GRAB_H__

#include "PCHand.h"
#include "PCPolygon.h"
#include "PCPolyhedron.h"
#include "Photo.h"
#include <list>

using namespace mapinect;

namespace photo {
	class Grab {
		public:
			Grab(Photo* photo, PCHand* hand);
			
			bool			Update(PCHand* hand);
			void			draw();
		
			Photo*			photo;
			list<PCHand*>	hands;
		};
}

#endif	// GRAB_H__
