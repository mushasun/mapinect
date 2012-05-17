#ifndef I_APPLICATION_H__
#define I_APPLICATION_H__

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

		virtual void setup() = 0;
		virtual void update() = 0;
		virtual void draw() = 0;
		virtual void exit() = 0;

		virtual void keyPressed(int key) = 0;
		virtual void mouseMoved(int x, int y) = 0;
		virtual void mouseDragged(int x, int y, int button) = 0;
		virtual void mousePressed(int x, int y, int button) = 0;
		virtual void mouseReleased(int x, int y, int button) = 0;
		virtual void windowResized(int w, int h) = 0;

		virtual void debugDraw() = 0;

		virtual void objectDetected(const IObject*)								{ }
		virtual void objectLost(const IObject*)									{ }
		virtual void objectMoved(const IObject*, const DataMovement&)			{ }
		virtual void objectTouched(const IObject*, const vector<DataTouch>&)	{ }

		/// <summary>
		/// Interface for handling texture. Loading, binding and enabling
		/// should be called through this object.
		/// </summary>
		ITxManager*		txManager;

		IArmController*	armController;
	};
}

#endif	// I_APPLICATION_H__
