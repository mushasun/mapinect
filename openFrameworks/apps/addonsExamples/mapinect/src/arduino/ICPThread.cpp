#include "ICPThread.h"

#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "Feature.h"
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

	void ICPThread::setup(Arduino* arduino) {
		reset();

		this->arduino = arduino;

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

		if (!IsFeatureICPActive())
		{
			// 1 - Re detectar la mesa
			pcl::ModelCoefficients coefficients;
			PCPtr detectedTableCloud;
			bool tableDetected = detectNewTable(afterMoving, coefficients, detectedTableCloud);

			// 2 - Si se detectó la nueva mesa, calcular ajuste necesario 
			if (tableDetected) 
			{
				saveCloud("nuevaMesaSinTransformar.pcd",*detectedTableCloud);
	
				// 2.1 - Calcular rotaciones para que tenga normal (0,-1,0)
				Eigen::Affine3f rotation = Table::calculateRotations(coefficients);

				ofVec3f centroidPrev  = computeCentroid(detectedTableCloud);
				Eigen::Affine3f correctedRotation;
				correctedRotation = getTranslationMatrix(centroidPrev) * (rotation * getTranslationMatrix(-centroidPrev));

				// Aplicar la rotación a la nube de mesa detectada, y actualizar los coeficientes con la nueva normal de la mesa
				detectedTableCloud = transformCloud(detectedTableCloud,correctedRotation);	

				ofVec3f newCentroid = computeCentroid(detectedTableCloud);
				cout << "centroide = " << newCentroid << endl;

				ofVec3f tableModelCentroid = computeCentroid(gModel->getTable()->getCloud());

				// Se debe mantener la misma altura de la mesa del modelo
				float Y = tableModelCentroid.y;
				float translateY = Y - computeCentroid(detectedTableCloud).y;
				Eigen::Affine3f translationY;
				translationY = Eigen::Translation<float, 3>(0.0, translateY, 0.0);
				detectedTableCloud = transformCloud(detectedTableCloud,translationY);	

				coefficients.values[0] = 0;
				coefficients.values[1] = -1;
				coefficients.values[2] = 0;
				coefficients.values[3] = Y;

				// 2.2 - Calcular traslación para que coincidan los nuevos vértices detectados con los vértices calibrados de la mesa
				vector<ofVec3f> tableModelVertexs(gModel->getTable()->getPolygonModelObject()->getMathModel().getVertexs());
				// Calcular la mesa con el nuevo plano, y nuevos vértices, ajustando los que se tenía del modelo
				TablePtr newTable = Table::updateTablePlane(coefficients,detectedTableCloud);
						// Matchear solo si son vértices "reales" y no estimados => y qué hacemos si son estimados?? Capaz los debería matchear igual, y dsp al volver a mover la mesa se van a volver a ajustar, pero lo importante es mantener el plano de la mesa, por la detección de los objetos
				// Obtener los nuevos vértices después de recalcular la mesa
				vector<ofVec3f> newTableVertexs(newTable->getPolygonModelObject()->getMathModel().getVertexs());

				// Tomar la pareja de vértices (A,A') que se corresponden, para calcular la traslación
				ofVec3f vertexA = tableModelVertexs.at(0);
				ofVec3f newVertexA = newTableVertexs.at(0);
				Eigen::Affine3f translationToFitVertexs = Eigen::Affine3f::Identity();

				if (vertexA.distance(newVertexA) < 0.50) {			
					ofVec3f trans = newVertexA - vertexA;	// A - A'
					translationToFitVertexs = Eigen::Translation<float, 3>(-trans.x, -trans.y, -trans.z);
				} else {
					cout << "da vuelta el X" << endl;
				}

				detectedTableCloud = transformCloud(detectedTableCloud,translationToFitVertexs);	
				saveCloud("nuevaMesaTransformada.pcd",*detectedTableCloud);
					
				// 2.3 - Calcular la transformación de ajuste necesaria, en base a las rotaciones y traslaciones halladas
				Eigen::Affine3f composedMatrix = translationToFitVertexs * (translationY * correctedRotation);
				gTransformation->setWorldTransformation(composedMatrix * gTransformation->getWorldTransformation());

				// Enviar evento que el brazo se dejó de mover, y ya está estable la transformación
				EventManager::addEvent(MapinectEvent(kMapinectEventTypeArmStoppedMoving));

				// Una vez que se terminó de actualizar la transformación, 
				//	libero el mutex para que puedan invocar al método getCloud
				gTransformation->cloudMutex.unlock();

				// Además, se debe volver a dibujar en la ventana de mapping
				gTransformation->setIsWorldTransformationStable(true);

			}
			else
			{
				// 2.B - Qué pasa si no se detectó la mesa?
				// resetear a la posicion inicial el brazo
				this->arduino->reset(true);
			}
		}
		else
		{
			pcl::ModelCoefficients coefficients;
			bool ICPneeded = false;

			// 1 - Verificar a partir de la nueva nube si la transformación teórica es buena
			// Para saber si es una buena estimación
			PCPtr planeCloud;
			float optimalAngleThreshold = 2;		// Tolerancia de angulo para la normal de la nueva mesa detectada = 2 grados
			float maxAngleThreshold = 30;			// Tolerancia máxima para el ángulo entre la normal del plano del modelo y la nueva normal, para encontrar al plano de la mesa
			float optimalPlaneDistance = 0.01;		// Tolerancia de distancia entre el plano de la mesa del modelo y la detectada = 1.0 cms
			bool isTableWellEstimated = findNewTablePlane(afterMoving, maxAngleThreshold, optimalAngleThreshold, optimalPlaneDistance, coefficients, planeCloud);

			if (!isTableWellEstimated || planeCloud->size() <= 0) 
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
					PCPtr icpPlaneCloud;
					pcl::ModelCoefficients icpCoefficients;
					isTableWellEstimated = findNewTablePlane(nubeAfterMovingTransfICP, maxAngleThreshold, optimalAngleThreshold, optimalPlaneDistance, icpCoefficients, icpPlaneCloud);
					if (isTableWellEstimated) //|| icpPlaneCloud->size() > 0) 
					{
						cout << "La transformacion de ICP es buena, se aplica" << endl;
						gTransformation->setWorldTransformation(icpTransf * gTransformation->getWorldTransformation());
						afterMoving = transformCloud(afterMoving, icpTransf);
						planeCloud = icpPlaneCloud;
						coefficients = pcl::ModelCoefficients(icpCoefficients);
					}
				}
			}


			// 3 - Re detectar la mesa
	//		PCPtr detectedTableCloud;
	//		bool tableDetected = detectNewTable(afterMoving, coefficients, detectedTableCloud);		
	
			// 4 - Actualizar el modelo con la nueva mesa detectada. Se actualiza el plano de la mesa y los vértices.
	//		if (tableDetected) 
			if (planeCloud->size() > 0 && isTableWellEstimated)
			{
				cout << "Se detectó un nuevo plano de mesa" << endl;
				// Ajustar el modelo de la mesa con el nuevo plano, y ajustar los vértices
	//			TablePtr table = Table::updateTablePlane(coefficients,detectedTableCloud);
				TablePtr table = Table::updateTablePlane(coefficients,planeCloud);
				gModel->setTable(table);

				// Una vez que se terminó de aplicar ICP y se actualizó la matriz de transformación, 
				//	libero el mutex para que puedan invocar al método getCloud
				gTransformation->cloudMutex.unlock();

				// Además, se debe volver a dibujar en la ventana de mapping
				gTransformation->setIsWorldTransformationStable(true);

			}
			else
			{
				this->arduino->reset(true);	// forceReset = true, esto es, el cloudMutex ya está en lock al entrar al método reset
			}
		}
	}

	bool ICPThread::findNewTablePlane(const PCPtr& cloud, float maxAngleThreshold, float optimalAngleThreshold, float maxDistance, pcl::ModelCoefficients& coefficients, PCPtr& planeCloud) {
		if (gModel->getTable() == NULL)
		{
			cout << "El modelo no tiene una mesa seteada" << endl;
			return false;
		} else if (cloud == NULL) {
			cout << "La nube pasada al findNewTablePlane es vacía" << endl;
			return false;
		}

		ofVec3f tableNormal = gModel->getTable()->getNormal();
		ofVec3f tableCentroid = gModel->getTable()->getCenter();
		Plane3D tablePlane(gModel->getTable()->getCoefficients());

		float distanceThreshold = 0.005; //  How close a point must be to the model in order to be considered an inlier
		planeCloud = findPlaneGivenNormal(cloud, coefficients, tableNormal, maxAngleThreshold, distanceThreshold);

		// Separar en clusters, para quedarme solo con los puntos de la mesa
		std::vector<pcl::PointIndices> clusterIndices =
				findClusters(planeCloud, Constants::TABLE_CLUSTER_TOLERANCE(), Constants::TABLE_CLUSTER_MIN_SIZE());
		float minDistanceToCentroid = MAX_FLOAT;
		for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
		{
			cout << "Hay clusters" << endl;
			PCPtr cloudCluster = getCloudFromIndices(planeCloud, *it);
			ofVec3f ptoCentroid = computeCentroid(cloudCluster);
			if (ptoCentroid.squareLength() < minDistanceToCentroid)
			{
				minDistanceToCentroid = ptoCentroid.squareLength();
				planeCloud = cloudCluster;
			}
		}

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

		if (planeCloud->size() > 0 && normalDifference < optimalAngleThreshold && centroidDistance < maxDistance) {
			return true;
		} else {
			return false;
		}
	}

	bool ICPThread::detectNewTable(const PCPtr& cloud, pcl::ModelCoefficients& newTableCoefficients,
		PCPtr& newTableCloud) {
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

		const Plane3D currentTablePlane(gModel->getTable()->getCoefficients());
		const float tableMinSize = Constants::TABLE_CLUSTER_MIN_SIZE();
		float minTablePlaneDistance = MAX_FLOAT;
		PCPtr remainingCloud(new PC());
		PCPtr cluster(tableCluster);
		do
		{
			pcl::ModelCoefficients coefficients;
			PCPtr planeCloud = extractBiggestPlane(cluster, coefficients, remainingCloud, 0.009,
				tableMinSize);
			if (planeCloud->size() == 0)
			{
				break;
			}
			else
			{
				if (currentTablePlane.distance(computeCentroid(planeCloud)) < minTablePlaneDistance)
				{
					newTableCoefficients = coefficients;
					newTableCloud = planeCloud;
				}
				cluster = remainingCloud;
			}
		}
		while (remainingCloud->size() > tableMinSize);
		
		return minTablePlaneDistance < MAX_FLOAT;
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