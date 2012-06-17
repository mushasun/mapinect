#ifndef CANVAS_H__
#define CANVAS_H__

#include "IMapper.h"

#include "IDrawer.h"
#include "IPolygon.h"
#include "TextureMapper2D.h"
#include "ofxCairoTexture.h"
#include <map>

using namespace mapinect;
using namespace std;

#include "ITxManager.h"

namespace drawing
{
	class Canvas : public IMapper
	{
	public:
		Canvas(ITxManager*, const IPolygonPtr& polygon, int backColor, int foreColor);
		virtual ~Canvas();

		void				draw();
		void				touchEvent(const DataTouch&);

		inline ofVec2f		mapToTexture(const ofVec3f& p) const				{ return texMapper.map(p); }

		inline int			getBackColor() const					{ return backColor; }
		void				setBackColor(int);
		inline int			getForeColor() const					{ return foreColor; }
		void				setForeColor(int);

	private:
		IPolygonPtr			polygon;
		ofxCairoTexture		texture;
		int					backColor;
		int					foreColor;
		ofVec2f				dimensions;
		vector<ofVec2f>		texCoords;
		TextureMapper2D		texMapper;

		map<int, IDrawer*>	drawers;
		GLuint				textureID;
	};
}

#endif	// CANVAS_H_

