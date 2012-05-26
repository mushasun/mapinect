#ifndef MAPINECT_POLYGON3D_H__
#define MAPINECT_POLYGON3D_H__

#include "ofVecUtils.h"
#include "Plane3D.h"
#include "Line3D.h"
#include "Line2D.h"

namespace mapinect
{
	class Polygon3D
	{
	public:
		Polygon3D() { }
		Polygon3D(const vector<ofVec3f>& vertexs);
		Polygon3D(const Plane3D& plane, const vector<ofVec3f>& vertexs);
		Polygon3D(const Polygon3D& polygon);
		virtual ~Polygon3D() { };

		inline const vector<ofVec3f>&	getVertexs() const		{ return vertexs; }
		inline const Plane3D&			getPlane() const		{ return plane; }
		inline const vector<Line3D>&	getEdges() const		{ return edges; }

		float				distance(const ofVec3f& p) const;
		ofVec3f				project(const ofVec3f& p) const;
		float				calculateArea() const;

		bool				isInPolygon(const ofVec3f& p) const;

	private:
		void				init();

		vector<ofVec3f>		vertexs;
		vector<ofVec2f>		vertexs2d;
		DiscardCoordinate	discardCoord;
		Plane3D				plane;
		vector<Line3D>		edges;
		vector<Line2D>		edges2d;

	};
}

#endif	// MAPINECT_POLYGON3D_H__
