#include "ICPThread.h"

#include <pcl/registration/icp.h>
#include "Globals.h"


using namespace std;

#define WAIT_TIME_MS		20

namespace mapinect {

	static unsigned long startTime; 

	ICPThread::ICPThread()	
	{
		icpMutex.lock();
		checkApplyICP = false;
		icpMutex.unlock();
	}

	void ICPThread::reset() 	
	{
		icpMutex.lock();
		checkApplyICP = false;
		icpMutex.unlock();
	}

	void ICPThread::setup() {
		reset();

		startThread(true, false);
	}

	//--------------------------------------------------------------
	void ICPThread::exit() {
		stopThread();
	}

	//--------------------------------------------------------------
	void ICPThread::threadedFunction() {
		while (isThreadRunning()) {
			if (lock()) {
				
				bool applyICP = false;
				{
					icpMutex.lock();
						if (checkApplyICP && !(cloudBeforeMoving.get() == NULL) && !(cloudAfterMoving.get() == NULL)) 
						{
							applyICP = true;
						}
					icpMutex.unlock();
				}

				if(applyICP)
				{
					processICP();
				}
				
				unlock();
				ofSleepMillis(WAIT_TIME_MS);
			}
		}
	}

	void ICPThread::applyICP(const PCPtr& cloudBefore, const PCPtr& cloudAfter) {
		icpMutex.lock();
			checkApplyICP = true;
			cloudBeforeMoving = cloudBefore;
			cloudAfterMoving = cloudAfter;
		icpMutex.unlock();
	}

	void ICPThread::processICP() {
		icpMutex.lock();
			checkApplyICP = false;
			pcl::PointCloud<PCXYZ>::Ptr beforeMoving (new pcl::PointCloud<PCXYZ>(*cloudBeforeMoving.get()));
			pcl::PointCloud<PCXYZ>::Ptr afterMoving  (new pcl::PointCloud<PCXYZ>(*cloudAfterMoving.get()));
		icpMutex.unlock();

		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputCloud(beforeMoving);
		icp.setInputTarget(afterMoving);

		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		//icp.setMaxCorrespondenceDistance (0.05);
		// Set the maximum number of iterations (criterion 1)
		//icp.setMaximumIterations (5);
		// Set the transformation epsilon (criterion 2)
		//icp.setTransformationEpsilon (1e-8);
		// Set the euclidean distance difference epsilon (criterion 3)
		//icp.setEuclideanFitnessEpsilon (1);

		pcl::PointCloud<pcl::PointXYZ> Final;

		startTime = ofGetSystemTime();

		icp.align(Final);

		unsigned int elapsedTime = (unsigned int) (ofGetSystemTime() - startTime);
		cout << "ICP time: " << elapsedTime << endl;

		if (icp.hasConverged())
		{
			cout << "ICP has converged with fitness score: " << icp.getFitnessScore() << endl;
			Eigen::Affine3f newTransf (icp.getFinalTransformation());
					
			gTransformation->setWorldTransformation(gTransformation->getWorldTransformation() * newTransf);
		}

		// Una vez que se terminó de aplicar ICP y se actualizó la matriz de transformación, 
		//	libero el mutex para que puedan invocar al método getCloud
		gTransformation->cloudMutex.unlock();
	}


}