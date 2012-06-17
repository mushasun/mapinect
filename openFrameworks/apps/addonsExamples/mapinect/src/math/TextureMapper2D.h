#ifndef MAPINECT_TEXTUREMAPPER2D_H__
#define MAPINECT_TEXTUREMAPPER2D_H__

#include "Polygon3D.h"

namespace mapinect
{
	class TextureMapper2D
	{
	public:
		TextureMapper2D() { }
		TextureMapper2D(const Polygon3D& polygon, const vector<ofVec2f>& texCoords, int originVertex);
		virtual ~TextureMapper2D() { }

		ofVec2f				map(const ofVec3f& p) const;

	private:
		Polygon3D			polygon;
		vector<ofVec2f>		texCoords;
		vector<Line2D>		texEdges;
		int					originVertex;

	};
}

#endif	// MAPINECT_TEXTUREMANAGER2D_H__
