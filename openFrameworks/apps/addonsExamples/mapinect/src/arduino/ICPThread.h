#ifndef ICP_THREAD_H__
#define ICP_THREAD_H__

#include "mapinectTypes.h"
#include "ofThread.h"
#include "ofxMutex.h"
#include "Arduino.h"

namespace mapinect
{
	class Arduino;

	class ICPThread : ofThread
	{
	public:
		ICPThread();

		void						reset();
		void						setup(Arduino* arduino);
		void						exit();
		virtual void				threadedFunction();

		void						applyICP(const PCPtr& cloudBefore, const PCPtr& cloudAfter, int maxIterations);

		void						processICP();

	private:
		bool						checkApplyICP;
		PCPtr						cloudBeforeMoving;
		PCPtr						cloudAfterMoving;
		int							maxIterations;

		ofxMutex					icpMutex;	

		// buscar en la nube el plano que tenga una normal dada para verificar si la mesa se ajustó bien
		bool						findNewTablePlane(const PCPtr& cloud, float maxAngleThreshold, float optimalAngleThreshold, float maxDistance, 
											pcl::ModelCoefficients& coefficients, PCPtr& planeCloud);

		bool						icpProcessing(const PCPtr& inputCloud, const PCPtr& inputTarget, Eigen::Affine3f& newTransf, 
											float maxDistance = 0.20, int maxIterations = 30);

		// Re detectar mesa usando el extractBiggestPlane
		bool						detectNewTable(const PCPtr& cloud, pcl::ModelCoefficients& coefficients, PCPtr& planeCloud);

		Arduino*					arduino;

	};
}

#endif	// ICP_THREAD_H__