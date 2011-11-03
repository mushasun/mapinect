#ifndef MAPINECT_BOX_H__
#define MAPINECT_BOX_H__

#include "Polyhedron.h"

namespace mapinect {
	enum BoxVertex {
		kBoxVertexA = 0,
		kBoxVertexB,
		kBoxVertexC,
		kBoxVertexD,
		kBoxVertexE,
		kBoxVertexF,
		kBoxVertexG,
		kBoxVertexH
	};

	class Box : public Polyhedron {
	public:
		Box(const ofxVec3f& dimensions);
		virtual ~Box() { }
	
		const ofxVec3f& getVertex(BoxVertex v);

	};
}

#endif	// MAPINECT_BOX_H__