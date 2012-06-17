#ifndef CANVAS_H__
#define CANVAS_H__

#include "IMapper.h"

#include "DataTouch.h"
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
		Canvas(const IPolygonPtr& polygon, const ofColor& backColor, const ofColor& foreColor);
		virtual ~Canvas();

		void					draw();
		void					touchEvent(const DataTouch&);

		inline ofVec2f			mapToTexture(const ofVec3f& p) const	{ return texMapper.map(p); }

		inline const ofColor&	getBackColor() const					{ return backColor; }
		void					setBackColor(const ofColor&);
		inline const ofColor&	getForeColor() const					{ return foreColor; }
		void					setForeColor(const ofColor&);

	private:
		IPolygonPtr				polygon;
		ofxCairoTexture			texture;
		ofColor					backColor;
		ofColor					foreColor;
		ofVec2f					dimensions;
		vector<ofVec2f>			texCoords;
		TextureMapper2D			texMapper;

		map<int, IDrawer*>		drawers;
	};
}

#endif	// CANVAS_H_

