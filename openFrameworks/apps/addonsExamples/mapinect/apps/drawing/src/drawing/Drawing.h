#ifndef DRAWING_H__
#define DRAWING_H__

#include "IApplication.h"

#include <map>
#include "Canvas.h"

using namespace mapinect;

namespace drawing
{
	enum AppStatus
	{
		kAppStatusDrawing,
		kAppStatusMenuPopup,
		kAppStatusFollowingObject,
		kAppStatusInsertingPicture
	};

	class Drawing : public IApplication
	{
	public:
		Drawing();
		virtual ~Drawing();

		virtual void	setup();
		virtual void	update(float elapsedTime);
		virtual void	draw();

		virtual void	objectDetected(const IObjectPtr&);
		virtual void	objectUpdated(const IObjectPtr&);
		virtual void	objectLost(const IObjectPtr&);
		virtual void	objectMoved(const IObjectPtr&, const DataMovement&);
		virtual void	objectTouched(const IObjectPtr&, const DataTouch&);
		virtual void	buttonReleased(const IButtonPtr&, const DataTouch&);

	private:

		void			setAppStatus(AppStatus);

		void			createMenu();
		void			createPicture(const DataTouch&);

		Polygon3D		table;
		Canvas*			canvas;
		IObjectPtr		object;
		AppStatus		status;
	};
}

#endif	// DRAWING_H__
