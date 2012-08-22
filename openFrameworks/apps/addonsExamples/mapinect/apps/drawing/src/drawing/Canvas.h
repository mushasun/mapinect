#ifndef CANVAS_H__
#define CANVAS_H__

#include "DataTouch.h"
#include "IDrawer.h"
#include "IPolygon.h"
#include "TextureMapper2D.h"
#include "ofxCairoTexture.h"
#include <map>

using namespace mapinect;
using namespace std;

namespace drawing
{
	class Canvas
	{
	public:
		Canvas(const IPolygonPtr& polygon, const ofColor& backColor, const ofColor& foreColor);
		virtual ~Canvas();

		void					update(const IPolygonPtr& polygon);
		void					draw();
		void					touchEvent(const DataTouch&);

		inline ofVec2f			mapToTexture(const ofVec3f& p) const	{ return texMapper.map(p); }

		inline const ofColor&	getBackColor() const					{ return backColor; }
		void					setBackColor(const ofColor&);
		inline const ofColor&	getForeColor() const					{ return foreColor; }
		void					setForeColor(const ofColor&);

		void					redrawIfNecessary();
	private:
		void					redraw();
		bool					needsToRedraw;

		IPolygonPtr				polygon;
		ofxCairoTexture			texture;
		ofColor					backColor;
		ofColor					foreColor;
		ofVec2f					dimensions;
		vector<ofVec2f>			texCoords;
		TextureMapper2D			texMapper;

		map<int, IDrawer*>		drawers;
		map<int, DataTouch>		touchPoints;
	};
}

#endif	// CANVAS_H_

