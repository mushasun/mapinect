#ifndef MAPINECT_POLYGON_H__
#define MAPINECT_POLYGON_H__

#include "ModelObject.h"
#include "IPolygon.h"

#include "Polygon3D.h"

namespace mapinect {

	class Polygon;

	typedef boost::shared_ptr<Polygon> PolygonPtr;

	class Polygon : public ModelObject, public IPolygon {
		public:
			Polygon() : vertexs(), container(NULL)				{ }
			virtual ~Polygon()									{ }

			inline int						getId()				{ return ModelObject::getId(); }
			inline const ofVec3f&			getCenter()			{ return ModelObject::getCenter(); }
			inline const ofVec3f&			getScale()			{ return ModelObject::getScale(); }
			inline const ofVec3f&			getRotation()		{ return ModelObject::getRotation(); }
			inline const ofVec3f&			getNormal()			{ return normal; }

			inline const vector<ofVec3f>&	getVertexs()		{ return vertexs; }
			inline const IObject*			getContainer()		{ return container; }
			inline const IPolygonName&		getName()			{ return name; }

			inline const Polygon3D&			getMathPolygon()	const	{ return mathPolygon; }

			inline void			setContainer(IObject* object)	{ container = object; }

			void				setVertex(int vertexNum, const ofVec3f& v);
			void				setVertexs(const vector<ofVec3f>& v);
			void				sortVertexs();
			void				setName(const IPolygonName&);

			float				calculateArea();
			ofVec3f				project(const ofVec3f&);
			virtual void		draw();
		
		private:
			vector<ofVec3f>		vertexs;
			ofVec3f				normal;
			IPolygonName		name;
			IObject*			container;
			Polygon3D			mathPolygon;
	};
}

#endif	// MAPINECT_POLYGON_H__