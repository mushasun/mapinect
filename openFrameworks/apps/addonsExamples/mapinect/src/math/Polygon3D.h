#ifndef MAPINECT_POLYGON3D_H__
#define MAPINECT_POLYGON3D_H__

#include "DiscardCoordinate.h"
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

		inline const vector<ofVec3f>&	getVertexs() const				{ return vertexs; }
		inline const vector<ofVec2f>&	getVertexs2d() const			{ return vertexs2d; }
		inline const DiscardCoordinate&	getDiscardCoordinate() const	{ return discardCoord; }
		inline const Plane3D&			getPlane() const				{ return plane; }
		inline const vector<Line3D>&	getEdges() const				{ return edges; }
		inline const vector<Line2D>&	getEdges2d() const				{ return edges2d; }
		inline const ofVec3f&			getCentroid() const				{ return centroid; }

		void							setPlane(const Plane3D&);
		void							setVertex(int pos, const ofVec3f& v);
		virtual void					setVertexs(const vector<ofVec3f>& v);

		float				distance(const ofVec3f& p) const;
		ofVec3f				project(const ofVec3f& p) const;
		float				calculateArea() const;

		bool				isInPolygon(const ofVec3f& p) const;
		bool				isInPolygon(const Line3D& l) const;
		bool				operator==(const Polygon3D&) const;

	private:
		void				init();

		vector<ofVec3f>		vertexs;
		vector<ofVec2f>		vertexs2d;
		DiscardCoordinate	discardCoord;
		Plane3D				plane;
		vector<Line3D>		edges;
		vector<Line2D>		edges2d;
		ofVec3f				centroid;

	};
}

#endif	// MAPINECT_POLYGON3D_H__
