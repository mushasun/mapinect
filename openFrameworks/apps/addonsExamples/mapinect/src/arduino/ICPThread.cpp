#include "ICPThread.h"

#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "Globals.h"
#include "pointUtils.h"
#include "Constants.h"
#include "Plane3D.h"

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

	void ICPThread::applyICP(const PCPtr& cloudBefore, const PCPtr& cloudAfter, int iterations) {
		icpMutex.lock();
			checkApplyICP = true;
			cloudBeforeMoving = cloudBefore;
			cloudAfterMoving = cloudAfter;
			maxIterations = iterations;
		icpMutex.unlock();
	}

	void ICPThread::processICP() {

		int iter = 0;
		icpMutex.lock();
			checkApplyICP = false;
			pcl::PointCloud<PCXYZ>::Ptr beforeMoving (new pcl::PointCloud<PCXYZ>(*cloudBeforeMoving.get()));
			saveCloud("ICPbeforeMoving.pcd",*beforeMoving);
			pcl::PointCloud<PCXYZ>::Ptr afterMoving  (new pcl::PointCloud<PCXYZ>(*cloudAfterMoving.get()));
			saveCloud("ICPafterMoving.pcd",*afterMoving);
			iter = maxIterations;
		icpMutex.unlock();

		if (gModel->getTable() != NULL) {

			float optimalAngleThreshold = 2;		// Tolerancia de angulo para la normal de la nueva mesa detectada = 2 grados
			float optimalPlaneDistance = 0.005;		// Tolerancia de distancia entre el plano de la mesa del modelo y la detectada = 0.5 cms
			float maxAngleThreshold = 10;			// Tolerancia máxima de angulo para la normal de la nueva mesa detectada = 10 grados
			float maxPlaneDistance = 0.05;			// Tolerancia máxima de distancia entre el plano de la mesa del modelo y la detectada = 5 cms
			PCPtr planeCloud;
			pcl::ModelCoefficients coefficients;

			bool ajustarMesa = false;

			// Verificar si la transformación calculada de forma teórica es ya una buena estimación
			bool isTableWellEstimated = findNewTablePlane(afterMoving, optimalAngleThreshold, optimalPlaneDistance, coefficients, planeCloud);
			if (isTableWellEstimated) {
				cout << "La transformacion estimada es buena. No aplicar ICP" << endl;
			} else {
				bool existsTableToAdjust = findNewTablePlane(afterMoving, maxAngleThreshold, maxPlaneDistance, coefficients, planeCloud);
				if (existsTableToAdjust) {
					// No aplicar ICP, y ajustar la mesa
					cout << "Se detecto mesa, pero no es optima. Ajustarla" << endl;
					ajustarMesa = true;
				} else {
					// No se encontro un plano que cumpla que tenga normal con diferencia de angulo menor a 10 grados, ni distancia menor a 5 cms
					// Aplicar ICP
					Eigen::Affine3f icpTransf;
					bool icpHasConverged = icpProcessing(afterMoving, beforeMoving, icpTransf, 0.20, iter); 
					if (icpHasConverged) {				
						PCPtr nubeAfterMovingTransfICP = transformCloud(afterMoving, icpTransf);
						saveCloud("nubeAfterMovingTransfICP.pcd",*nubeAfterMovingTransfICP);
						cout << "Verificando resultado de ICP" << endl;
						bool isICPTableWellEstimated = findNewTablePlane(nubeAfterMovingTransfICP, optimalAngleThreshold, optimalPlaneDistance, coefficients, planeCloud);
						if (isICPTableWellEstimated) {
							cout << "La transformacion de ICP es buena, se aplica" << endl;
							gTransformation->setWorldTransformation(icpTransf * gTransformation->getWorldTransformation());
						} else {
							//El resultado de ICP no es óptimo. Ver si se puede ajustar.
							bool existsICPTableToAdjust = findNewTablePlane(nubeAfterMovingTransfICP, maxAngleThreshold, maxPlaneDistance, coefficients, planeCloud);
							if (existsICPTableToAdjust) {
								cout << "Tras ICP se detecto mesa, pero no es optima. Ajustarla" << endl;
								ajustarMesa = true;
							} else {
								cout << "Tras ICP no se pudo hallar la mesa. No se aplica la transf." << endl;				
							}
						}			
					} else {
						cout << "ICP no convergio" << endl;
					}
				}
			}

			if (ajustarMesa){
				gModel->tableMutex.lock();
					TablePtr modelTable = gModel->getTable();
					if (modelTable != NULL) {
						modelTable->updateTablePlane(coefficients, planeCloud);
						// Ver lo de los vertices, si hay que proyectarlos o qué
						cout << "Se pisa la mesa con la nueva detectada" << endl;
					}
				gModel->tableMutex.unlock();
			}
		}

		// Una vez que se terminó de aplicar ICP y se actualizó la matriz de transformación, 
		//	libero el mutex para que puedan invocar al método getCloud
		gTransformation->cloudMutex.unlock();

		// Además, se debe volver a dibujar en la ventana de mapping
		gTransformation->setIsWorldTransformationStable(true);

	}

	bool ICPThread::findNewTablePlane(const PCPtr& cloud, float maxAngleThreshold, float maxDistance, pcl::ModelCoefficients& coefficients, PCPtr& planeCloud) {
		ofVec3f tableNormal = gModel->getTable()->getNormal();
		ofVec3f tableCentroid = gModel->getTable()->getCenter();
		Plane3D tablePlane(gModel->getTable()->getCoefficients());

		float distanceThreshold = 0.005; //  How close a point must be to the model in order to be considered an inlier
		planeCloud = findPlaneGivenNormal(cloud, coefficients, tableNormal, maxAngleThreshold, distanceThreshold);

		Plane3D planeFound(coefficients);
		float centroidDistance = planeFound.distance(tableCentroid);
		ofVec3f planeNormal = planeFound.getNormal();
		float normalDifference = abs(planeNormal.angle(tablePlane.getNormal()));

		cout << "Diferencia de Angulo entre Normales: " << normalDifference << " y maximo vale: " << maxAngleThreshold << endl;
		cout << "Distancia entre planos es: " << centroidDistance << " y maxima es: " << maxDistance << endl;

		// A veces la normal forma casi 180 grados con la buscada, es decir apunta en el sentido opuesto, pero es la mesa
		if (abs(180 - normalDifference) < maxAngleThreshold) {
			normalDifference = abs(180 - normalDifference);	
		}

		if (planeCloud->size() > 0 && normalDifference < maxAngleThreshold && centroidDistance < maxDistance) {
			return true;
		} else {
			return false;
		}
	}

	bool ICPThread::icpProcessing(const PCPtr& inputCloud, const PCPtr& inputTarget, Eigen::Affine3f& newTransf, 
			float maxDistance, int maxIterations) {
		cout << "Aplicando ICP..." << endl;
		// Comienza procesamiento de ICP
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputCloud(inputCloud);
		icp.setInputTarget(inputTarget);

		// Set the max correspondence distance (correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance (maxDistance * maxDistance);
		icp.setRANSACOutlierRejectionThreshold(maxDistance * maxDistance);
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations (maxIterations);
		// Set the transformation epsilon (criterion 2)
		//icp.setTransformationEpsilon (1e-8);
		// Set the euclidean distance difference epsilon (criterion 3)
		//icp.setEuclideanFitnessEpsilon (1);

		pcl::PointCloud<pcl::PointXYZ> Final;

		startTime = ofGetSystemTime();

		icp.align(Final);

		unsigned int elapsedTime = (unsigned int) (ofGetSystemTime() - startTime);
		cout << "Tiempo de ICP: " << elapsedTime << endl;

		if (icp.hasConverged())
		{
			cout << "ICP convergio con fitness score igual a: " << icp.getFitnessScore() << endl;
			newTransf  = icp.getFinalTransformation();
			return true;
		} else {
			cout << "ICP no convergio" << endl;
			return false;
		}		

	}

}