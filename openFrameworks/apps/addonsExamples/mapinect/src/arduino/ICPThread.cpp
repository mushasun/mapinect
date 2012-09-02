#include "ICPThread.h"

#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "Globals.h"
#include "pointUtils.h"
#include "transformationUtils.h"
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

		PCPtr detectedTableCloud;
		pcl::ModelCoefficients coefficients;
		bool ICPneeded = false;

		// 1 - Verificar a partir de la nueva nube si la transformación teórica es buena
		// Para saber si es una buena estimación
		PCPtr planeCloud;
		float optimalAngleThreshold = 2;		// Tolerancia de angulo para la normal de la nueva mesa detectada = 2 grados
		float optimalPlaneDistance = 0.005;		// Tolerancia de distancia entre el plano de la mesa del modelo y la detectada = 0.5 cms
		bool isTableWellEstimated = findNewTablePlane(afterMoving, optimalAngleThreshold, optimalPlaneDistance, coefficients, planeCloud);
		if (!isTableWellEstimated) 
		{
			ICPneeded = true;
			cout << "La transformacion estimada no es buena; aplicar ICP" << endl;
		} else {
			cout << "No aplicar ICP" << endl;
		}

		// 2 - Si se necesita, aplicar ICP para corregir la transformación (y así corregir a los objetos)
		if (ICPneeded) 
		{
			Eigen::Affine3f icpTransf;
			bool icpHasConverged = icpProcessing(afterMoving, beforeMoving, icpTransf, 0.20, iter); 
			if (icpHasConverged) 
			{				
				PCPtr nubeAfterMovingTransfICP = transformCloud(afterMoving, icpTransf);
				saveCloud("nubeAfterMovingTransfICP.pcd",*nubeAfterMovingTransfICP);
				cout << "Verificando resultado de ICP" << endl;
				bool isICPTableWellEstimated = findNewTablePlane(nubeAfterMovingTransfICP, optimalAngleThreshold, optimalPlaneDistance, coefficients, planeCloud);
				if (isICPTableWellEstimated) 
				{
					cout << "La transformacion de ICP es buena, se aplica" << endl;
					gTransformation->setWorldTransformation(icpTransf * gTransformation->getWorldTransformation());
					afterMoving = transformCloud(afterMoving,icpTransf);
				}
			}
		}
		
		// 3 - Re detectar la mesa
		bool tableDetected = detectNewTable(afterMoving, coefficients, detectedTableCloud);		
	
		// 4 - Actualizar el modelo con la nueva mesa detectada. Se actualiza el plano de la mesa y los vértices.
		if (tableDetected) 
		{
			cout << "Se detectó un nuevo plano de mesa" << endl;
			// Ajustar el modelo de la mesa con el nuevo plano, y ajustar los vértices
			Table::updateTablePlane(coefficients,detectedTableCloud);
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

	bool ICPThread::detectNewTable(const PCPtr& cloud, pcl::ModelCoefficients& coefficients, PCPtr& planeCloud) {
		if (cloud->size() == 0)
		{
			return false;
		}

		saveCloud("nuevaNubeMesa.pcd",*cloud);

		std::vector<pcl::PointIndices> clusterIndices =
			findClusters(cloud, Constants::TABLE_CLUSTER_TOLERANCE(), Constants::TABLE_CLUSTER_MIN_SIZE());

		PCPtr tableCluster;
		float minDistanceToCentroid = MAX_FLOAT;
		for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
		{
			PCPtr cloudCluster = getCloudFromIndices(cloud, *it);
			ofVec3f ptoCentroid = computeCentroid(cloudCluster);
			if (ptoCentroid.squareLength() < minDistanceToCentroid)
			{
				minDistanceToCentroid = ptoCentroid.squareLength();
				tableCluster = cloudCluster;
			}
		}

		saveCloud("tableCluster.pcd",*tableCluster);

		PCPtr result(new PC());
		planeCloud = extractBiggestPlane(tableCluster, coefficients, result, 0.009);
		
		return true;
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