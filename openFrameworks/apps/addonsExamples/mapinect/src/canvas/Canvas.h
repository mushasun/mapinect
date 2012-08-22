#ifndef CANVAS_H__
#define CANVAS_H__

#include "DataTouch.h"
#include "IDrawer.h"
#include "IPolygon.h"
#include "TextureMapper2D.h"
#include "ofxCairoTexture.h"
#include <map>

using namespace std;

namespace mapinect
{
	class Canvas
	{
	public:
		Canvas(int polygonId, const Polygon3D& polygon, int width, int height, const ofColor& backColor, const ofColor& foreColor);
		virtual ~Canvas();

		void					update(const Polygon3D& polygon);
		void					draw();
		void					touchEvent(const DataTouch&);

		inline const ofColor&	getBackColor() const					{ return backColor; }
		void					setBackColor(const ofColor&);
		inline const ofColor&	getForeColor() const					{ return foreColor; }
		void					setForeColor(const ofColor&);

		void					redrawIfNecessary();
	private:
		void					redraw();
		bool					needsToRedraw;

		int						polygonId;
		Polygon3D				polygon;
		int						width;
		int						height;
		ofColor					backColor;
		ofColor					foreColor;
		ofxCairoTexture			texture;
		vector<ofVec2f>			texCoords;
		TextureMapper2D			texMapper;

		map<int, IDrawer*>		drawers;
		map<int, DataTouch>		touchPoints;
	};
}

#endif	// CANVAS_H_

