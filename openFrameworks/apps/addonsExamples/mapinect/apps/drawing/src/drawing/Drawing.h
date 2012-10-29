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
		kAppStatusInsertingPicture,
		kAppStatusPositioningPicture
	};

	enum AppAction
	{
		kAppActionPickColor = 0,
		kAppActionFollowObject,
		kAppActionInsertPicture,
		kAppActionConfirmPicture
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
		virtual void		buttonPressed(const IButtonPtr&, const DataTouch&);
		virtual void		buttonReleased(const IButtonPtr&, const DataTouch&);

		virtual void		keyPressed(int key);

	private:

		void				setAppStatus(AppStatus);

		vector<ofVec3f>		polygonOnTable(const ofVec3f& center, float xLength, float zLength, float elevation, float radius, float rotation);
		void				clearActions();
		void				createMenu(const ofVec3f&);
		void				destroyMenu();
		void				createPalette(const ofVec3f&);
		void				destroyPalette();
		void				createPicture(const ofVec3f&);
		void				confirmPicture();

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
		vector<ofImage*>	paletteTextures;
		vector<ofFloatColor>paletteColors;

		IButtonPtr			pictureCurrent;
		ofImage*			pictureTextureCurrent;
	};
}

#endif	// DRAWING_H__
