#ifndef DRAWING_H__
#define DRAWING_H__

#include "IApplication.h"

#include <map>
#include "Canvas.h"
#include "ofSoundPlayer.h"

using namespace mapinect;

namespace drawing
{
	enum AppStatus
	{
		kAppStatusDrawing,
		kAppStatusPickingColor,
		kAppStatusFollowingObject,
		kAppStatusInsertingPicture
	};

	enum AppAction
	{
		kAppActionPickColor = 0,
		kAppActionFollowObject,
		kAppActionInsertPicture
	};

	enum DrawingMode
	{
		kDrawingModePen,
		kDrawingModeFill
	};

	class Drawing : public IApplication
	{
	public:
		Drawing();
		virtual ~Drawing();

		virtual void		setup();
		virtual void		update(float elapsedTime);
		virtual void		draw();

		virtual void		objectDetected(const IObjectPtr&);
		virtual void		objectUpdated(const IObjectPtr&);
		virtual void		objectLost(const IObjectPtr&);
		virtual void		objectMoved(const IObjectPtr&, const DataMovement&);
		virtual void		objectTouched(const IObjectPtr&, const DataTouch&);
		virtual void		pointTouched(const DataTouch& touchPoint);
		virtual void		buttonReleased(const IButtonPtr&, const DataTouch&);

	private:

		void				setAppStatus(AppStatus);

		vector<ofVec3f>		polygonOnTable(const ofVec3f& center, float xLength, float zLength, float elevation, float radius, float rotation);
		void				clearActions();
		void				createMenu(const ofVec3f&);
		void				destroyMenu();
		void				createPalette(const ofVec3f&);
		void				destroyPalette();
		void				createPicture(const ofVec3f&);

		Polygon3D			table;
		map<int, DataTouch>	touchPoints;
		Canvas*				canvas;
		IObjectPtr			object;
		AppStatus			status;
		ofSoundPlayer		menuSound;

		map<int, int>		actions;

		float				menuTimer;
		bool				menuVisible;
		vector<ofImage*>	textures;

		bool				paletteVisible;
		vector<ofFloatColor>paletteColors;
	};
}

#endif	// DRAWING_H__
