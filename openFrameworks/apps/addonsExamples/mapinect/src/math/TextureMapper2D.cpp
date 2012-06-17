#include "TextureMapper2D.h"

#include "utils.h"

namespace mapinect
{
	TextureMapper2D::TextureMapper2D(const Polygon3D& polygon, const vector<ofVec2f>& texCoords, int originVertex)
		 : polygon(polygon), texCoords(texCoords), originVertex(originVertex)
	{
		assert(polygon.getVertexs().size() == texCoords.size());
		assert(inRange(originVertex, 0, (int)polygon.getVertexs().size() - 1));
		for (int i = 0; i < texCoords.size(); ++i)
		{
			texEdges.push_back(Line2D(texCoords[i], texCoords[(i + 1) % texCoords.size()]));
		}
	}

	ofVec2f TextureMapper2D::map(const ofVec3f& p) const
	{
		float k[2] = { 0, 0 };
		int kix[2] = { -1, -1 };
		int ix = 0, i = 0;
		for (vector<Line3D>::const_iterator l = polygon.getEdges().begin(); l != polygon.getEdges().end(); ++i, ++l)
		{
			float lk = l->projectedK(p);
			if (l->isInSegment(lk))
			{
				k[ix] = lk;
				kix[ix++] = i;
				if (ix == 2)
					break;
			}
		}
		assert(ix == 2);
		Line2D line0 = texEdges[kix[0]].parallelLineThrough(texEdges[kix[1]].calculateValue(k[kix[1]]));
		Line2D line1 = texEdges[kix[1]].parallelLineThrough(texEdges[kix[0]].calculateValue(k[kix[0]]));

		return line0.intersection(line1);
	}

}
