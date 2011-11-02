#ifndef MAPINECT_POLYGON_H__
#define MAPINECT_POLYGON_H__

#include "ModelObject.h"

#include "ofxVec3f.h"
#include <vector>

namespace mapinect {

	class Polygon : public ModelObject {
		public:
			Polygon() : vertexs() { }
			virtual ~Polygon() { }

			const ofxVec3f&		getVertex(int vertexNum);
			void				addVertex(ofxVec3f);
			inline int			getVertexCount()			{ return vertexs.size(); }

			void				sortVertexs();

			float				calculateArea();
			void				resetVertex();
			virtual void draw();
		
		private:
			vector<ofxVec3f>	vertexs;
			ofxVec3f			normal;
	};
}

#endif	// MAPINECT_POLYGON_H__