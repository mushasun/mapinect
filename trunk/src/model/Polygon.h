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
			void				setVertex(int vertexNum, const ofxVec3f& v);
			void				addVertex(const ofxVec3f&);
			inline int			getVertexCount()			{ return vertexs.size(); }
			inline const vector<ofxVec3f>&	getVertexs()	{ return vertexs; }
			inline const ofxVec3f&	getNormal()				{ return normal; }

			void				sortVertexs();

			float				calculateArea();
			ofxVec3f			project(const ofxVec3f&);
			void				resetVertex();
			virtual void		draw();
		
		private:
			vector<ofxVec3f>	vertexs;
			ofxVec3f			normal;
	};
}

#endif	// MAPINECT_POLYGON_H__