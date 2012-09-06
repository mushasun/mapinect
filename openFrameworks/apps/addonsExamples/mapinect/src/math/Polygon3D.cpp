#include "Polygon3D.h"

namespace mapinect
{

	Polygon3D::Polygon3D(const vector<ofVec3f>& vertexs)
		: vertexs(vertexs), plane(vertexs[0],vertexs[1],vertexs[2])
	{
		assert(vertexs.size() >= 2);
		init();
	}

	Polygon3D::Polygon3D(const Plane3D& plane, const vector<ofVec3f>& vs)
		: plane(plane)
	{
		for (int i = 0; i < vs.size(); i++)
		{
			vertexs.push_back(plane.project(vs[i]));
		}
		init();
	}

	Polygon3D::Polygon3D(const Polygon3D& polygon)
		: vertexs(polygon.vertexs), plane(polygon.plane)
	{
		init();
	}

	void Polygon3D::init()
	{
		discardCoord = calculateDiscardCoordinate(vertexs);
		vertexs2d.clear();
		for (vector<ofVec3f>::const_iterator it = vertexs.begin(); it != vertexs.end(); ++it)
		{
			vertexs2d.push_back(discardCoordinateOfVec3f(*it, discardCoord));
		}
		edges.clear();
		edges2d.clear();
		for (int i = 0; i < vertexs.size(); i++)
		{
			edges.push_back(Line3D(vertexs[i], vertexs[(i + 1) % vertexs.size()]));
			edges2d.push_back(Line2D(vertexs2d[i], vertexs2d[(i + 1) % vertexs.size()]));
		}
		centroid = computeCentroid(vertexs);
	}

	void Polygon3D::setPlane(const Plane3D& p)
	{
		plane = p;
	}

	void Polygon3D::setVertex(int pos, const ofVec3f& v)
	{
		assert(pos <= vertexs.size());
		//assert(plane.distance(v) < MATH_EPSILON);
		vertexs[pos] = v;
		init();
	}

	void Polygon3D::setVertexs(const vector<ofVec3f>& newVertexs)
	{
		vertexs = newVertexs;
		init();
	}

	float Polygon3D::distance(const ofVec3f& p) const
	{
		return p.distance(project(p));
	}

	ofVec3f Polygon3D::project(const ofVec3f& p) const
	{
		ofVec3f planeProjected(plane.project(p));

		if (isInPolygon(planeProjected))
			return planeProjected;

		float minDistance = MAX_FLOAT;
		ofVec3f result;
		for (int i = 0; i < edges.size(); i++)
		{
			float k = edges[i].projectedK(p);
			ofVec3f edgeProjected = edges[i].calculateValue(k);
			float distance = p.distance(edgeProjected);
			if (edges[i].isInSegment(k) && distance < minDistance)
			{
				minDistance = distance;
				result = edgeProjected;
			}
		}
		for (int i = 0; i < vertexs.size(); i++)
		{
			float distance = p.distance(vertexs[i]);
			if (distance < minDistance)
			{
				minDistance = distance;
				result = vertexs[i];
			}
		}

		return result;
	}

	bool Polygon3D::isInPolygon(const ofVec3f& point) const
	{
		bool result = true;
		
		result = plane.distance(point) <= MATH_EPSILON_2;

		/*if(!result)
			cout << "not for distance: " << plane.distance(point) << endl;*/
		for (int i = 0; result && i < edges.size(); i++)
			result = edges[i].isInSegment(edges[i].projectedK(point));
		
		return result;
	}

	bool Polygon3D::isInPolygon(const Line3D& l) const
	{
		ofVec3f intersection = plane.intersection(l);
		if (intersection != BAD_OFVEC3F)
		{
			return isInPolygon(intersection);
		}
		return false;
	}

	bool Polygon3D::operator==(const Polygon3D& other) const
	{
		if (vertexs.size() == other.vertexs.size())
			for (int i = 0; i < vertexs.size(); i++)
				if (vertexs[i].distance(other.vertexs[i]) > MATH_EPSILON)
					return false;
			return true;
		return false;
	}

	float Polygon3D::calculateArea() const
	{
		// Code taken from http://softsurfer.com/Archive/algorithm_0101/algorithm_0101.htm#area3D_Polygon()
		float area = 0;
		int   i, j, k;			// loop indices

		vector<ofVec3f> auxVertexs(vertexs);
		auxVertexs.push_back(auxVertexs.at(0));
		auxVertexs.push_back(auxVertexs.at(1));
		
		// compute area of the 2D projection
		for (i = 1, j = 2, k = 0; i <= auxVertexs.size() - 2; i++, j++, k++) {
			switch (discardCoord) {
			case kDiscardCoordinateX:
				area += (auxVertexs[i].y * (auxVertexs[j].z - auxVertexs[k].z));
				continue;
			case kDiscardCoordinateY:
				area += (auxVertexs[i].x * (auxVertexs[j].z - auxVertexs[k].z));
				continue;
			case kDiscardCoordinateZ:
				area += (auxVertexs[i].x * (auxVertexs[j].y - auxVertexs[k].y));
				continue;
			}
		}

		// scale to get area before projection
		switch (discardCoord) {
		case kDiscardCoordinateX:
			area *= (1.0f / (2.0f * plane.getNormal().x));
			break;
		case kDiscardCoordinateY:
			area *= (1.0f / (2.0f * plane.getNormal().y));
			break;
		case kDiscardCoordinateZ:
			area *= (1.0f / (2.0f * plane.getNormal().z));
		}
		return abs(area);
	}

}
