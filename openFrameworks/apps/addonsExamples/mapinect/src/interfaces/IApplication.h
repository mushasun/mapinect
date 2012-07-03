#ifndef I_APPLICATION_H__
#define I_APPLICATION_H__

#include "ofEvents.h"

#include "INotification.h"
#include "DataMovement.h"
#include "DataTouch.h"
#include "IArmController.h"
#include "IObject.h"
#include "IButtonManager.h"

namespace mapinect {
	
	class IButtonManager;

	/// <summary>
	/// IApplication.h
	/// 
	/// Provides an interface for each application that uses mapinect.
	/// </summary>
	class IApplication: public INotification{
	public: 

		/// <summary>
		/// Override any of this methods to listen the event from user application
		/// </summary>

		virtual void exit()													{ }
		virtual void setup()												{ }
		virtual void update()												{ }
		virtual void draw()													{ }

		virtual void debugDraw()											{ }

		virtual void keyPressed(int key)									{ }
		virtual void keyReleased(int key)									{ }
		virtual void windowMoved(int x, int y)								{ }
		virtual void mouseMoved(int x, int y)								{ }
		virtual void mouseDragged(int x, int y, int button)					{ }
		virtual void mousePressed(int x, int y, int button)					{ }
		virtual void mouseReleased(int x, int y, int button)				{ }
		virtual void dragEvent(ofDragInfo info)								{ }

		IArmController*	armController;
	};
}

#endif	// I_APPLICATION_H__
