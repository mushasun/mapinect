#ifndef I_APPLICATION_H__
#define I_APPLICATION_H__

#include "ofEvents.h"

#include "DataMovement.h"
#include "DataTouch.h"
#include "IArmController.h"
#include "IObject.h"
#include "ITxManager.h"

namespace mapinect {
	
	/// <summary>
	/// IApplication.h
	/// 
	/// Provides an interface for each application that uses mapinect.
	/// </summary>
	class IApplication {
	public:

		virtual void exit() = 0;
		virtual void setup() = 0;
		virtual void update() = 0;
		virtual void draw() = 0;

		virtual void keyPressed(int key) = 0;
		virtual void keyReleased(int key) = 0;
		virtual void windowMoved(int x, int y) = 0;
		virtual void mouseMoved(int x, int y) = 0;
		virtual void mouseDragged(int x, int y, int button) = 0;
		virtual void mousePressed(int x, int y, int button) = 0;
		virtual void mouseReleased(int x, int y, int button) = 0;
		virtual void dragEvent(ofDragInfo info) = 0;

		virtual void debugDraw() = 0;

		virtual void objectDetected(const IObjectPtr&)						{ }
		virtual void objectUpdated(const IObjectPtr&)						{ }
		virtual void objectLost(const IObjectPtr&)							{ }
		virtual void objectMoved(const IObjectPtr&, const DataMovement&)	{ }
		virtual void objectTouched(const IObjectPtr&, const DataTouch&)		{ }

		/// <summary>
		/// Interface for handling texture. Loading, binding and enabling
		/// should be called through this object.
		/// </summary>
		ITxManager*		txManager;

		IArmController*	armController;
	};
}

#endif	// I_APPLICATION_H__
