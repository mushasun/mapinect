#include "Table.h"

#include "Constants.h"
#include "DataObject.h"
#include "Feature.h"
#include "Globals.h"
#include "mapinectTypes.h"
#include "pointUtils.h"
#include "transformationUtils.h"
#include <algorithm>
#include "SortPolar.h"
#include <cmath>

namespace mapinect
{
	vector<ofVec3f> reorderTableVertexs(const vector<ofVec3f>& vertexs)
	{
		// Se deben ordenar los vértices en screen coords
		//	de modo que el A sea el que está mas cerca del (0,0) en Screen coords
		//	y luego en sentido horario B, C y D
		//				   A *------------* D
		//					/			   \
		//				   /			    \
		//			    B *------------------* C
		vector<ofVec3f> vertexs2D;
		for (int i = 0; i < vertexs.size(); i++) 
		{
			ofVec3f screenCoord = getScreenCoords(vertexs.at(i));	// Transformar a 2D
			screenCoord.z = 0;										// Descartar coordenada Z
			vertexs2D.push_back(screenCoord);						// Agregar nuevo elemento al final del vector
		}
		// Re ordena el conjunto vertexs2D, en sentido antihorario
		sort(vertexs2D.begin(),vertexs2D.end(),SortPolar(vertexs2D));	
		// Buscar cuál es el vértice A, que es el que esté más cerca del (0,0) en coordenadas de pantalla
		ofVec3f init = ofVec3f(KINECT_DEFAULT_WIDTH,KINECT_DEFAULT_HEIGHT, 0); // En coordenadas de pantalla
		int indexAVertex2D = -1;
		float distanceSquared = init.lengthSquared();
		for (int i = 0; i < vertexs2D.size(); i++) 
		{
			// Calcular la distancia entre el punto actual y el (0,0)
			float currentDistanceSquared = vertexs2D[i].lengthSquared();
			if (currentDistanceSquared < distanceSquared) 
			{
				distanceSquared = currentDistanceSquared;
				indexAVertex2D = i;
			}
		}
		vector<ofVec3f> orderedVertexs2D;
		if (indexAVertex2D == -1)
		{
			cout << "Error - no se pudo determinar cual es el vertice A de la mesa" << endl;
		}
		else
		{
			// Queremos que orderedVertexs2D[0] = vertexs2D[indexAVertex2D] y mantener el orden;
			for (int i = 0; i < vertexs2D.size(); i++)
				orderedVertexs2D.push_back(vertexs2D[(indexAVertex2D + i) % vertexs2D.size()]);
		}

		vector<ofVec3f> orderedVertexs3D;	// orderedVertexs3D = {A,B,C,D}
		for (int i = 0; i < orderedVertexs2D.size(); i++)
			for (int j = 0; j < vertexs.size(); j++)
			{
				ofVec3f screenCoord = getScreenCoords(vertexs.at(j));	// Transformar a 2D
				screenCoord.z = 0;
				if (screenCoord == orderedVertexs2D.at(i))
				{
					orderedVertexs3D.push_back(vertexs.at(j));
					break;
				}
			}

		// Re ordenar para que los vertices queden en sentido horario, manteniendo al A primero
		vector<ofVec3f> clockwiseVertexs3D;
		clockwiseVertexs3D.resize(4);
		clockwiseVertexs3D.at(0) = orderedVertexs3D.at(0);
		for (int i = 0; i < 3; i++)
		{
			clockwiseVertexs3D.at(i + 1) = orderedVertexs3D.at(3 - i); 
		}

		return clockwiseVertexs3D;
	}

	bool isTableVertexInSafeArea(const ofVec3f& vertex)
	{
		const int TOLERANCE = Constants::PIXEL_TOLERANCE_ESTIMATED_VERTEX;

		const float minX = 0 + TOLERANCE;
		const float maxX = KINECT_DEFAULT_WIDTH - TOLERANCE;
		const float minY = 0 + TOLERANCE;
		const float maxY = KINECT_DEFAULT_HEIGHT - TOLERANCE;
		const ofVec3f screenVertex(getScreenCoords(vertex));

		return inRange(screenVertex.x, minX, maxX) && inRange(screenVertex.y, minY, maxY);
	}

	void Table::calibrateTable(TablePtr& table)
	{
		vector<ofVec3f> tableVertexs = table->getPolygonModelObject()->getMathModel().getVertexs();

		/************************************************************************/
		/*     Calibración inicial de la Mesa - Tamaño de la mesa en el XML  	*/
		const float TABLE_LENGTH_AB = Constants::TABLE_LENGTH_AB;
		const float TABLE_LENGTH_AD = Constants::TABLE_LENGTH_AD;

		const ofVec3f pWorldA	= tableVertexs.at(0);

		const ofVec3f pWorldB	= tableVertexs.at(1);
		ofVec3f nuevoVerticeB	= pWorldB;

		const ofVec3f pWorldC	= tableVertexs.at(2);
		ofVec3f nuevoVerticeC	= pWorldC;

		const ofVec3f pWorldD	= tableVertexs.at(3);
		ofVec3f nuevoVerticeD	= pWorldD;

		if (!isTableVertexInSafeArea(pWorldA)) {
			// Si está fuera de pantalla, es porque el Kinect no llega a ver a ese punto
			//	y por lo tanto fue estimado en la detección del rectángulo de la mesa
			cout << "El vertice A fue estimado" << endl;
		}  

		//       A -------- B -----B'
		//	    /		     \
		//     /			  \
		//    D----------------C
		//   /
		//  D'
		// TABLE_LENGTH_AB = dist(A,B')
		// TABLE_LENGTH_AD = dist(A,D')

		mapinect::Line3D lineAB(pWorldA, pWorldB);	// Linea entre A y B, ambos en 3D
		double distanceAB = pWorldA.distance(pWorldB);
		// Se va a modificar el vertice B solo si fue estimado
		if(!isTableVertexInSafeArea(pWorldB) || (TABLE_LENGTH_AB > distanceAB) ) {
			// B fue estimado, entonces se debe calcular el nuevo B' con el ancho de mesa 
			nuevoVerticeB = lineAB.calculateValue(TABLE_LENGTH_AB / distanceAB);
			cout << "El vertice B fue modificado" << endl;
		}

		mapinect::Line3D lineAD(pWorldA,pWorldD);	// Linea entre A y D, ambos en 3D
		double distanceAD = pWorldA.distance(pWorldD);
		// Se va a modificar el vertice D solo si fue estimado
		if(!isTableVertexInSafeArea(pWorldD) || (TABLE_LENGTH_AD > distanceAD) ) {
			// D fue estimado, entonces se debe calcular el nuevo D' con el largo de mesa 
			nuevoVerticeD = lineAD.calculateValue(TABLE_LENGTH_AD / distanceAD);
			cout << "El vertice D fue modificado" << endl;
			distanceAD=pWorldA.distance(nuevoVerticeD);
		}

		if (nuevoVerticeB != pWorldB || nuevoVerticeD != pWorldD ) 
		{
			// Si B o D fueron recalculados, se debe actualizar C también
			// AB + AD = AC
			// AC = C - A
			// => C = (AB + AD) + A = (B-A + D-A) + A
			ofVec3f AC = nuevoVerticeB - pWorldA + nuevoVerticeD - pWorldA; 
			nuevoVerticeC = AC + pWorldA;
			cout << "El vertice C fue modificado" << endl;

			vector<ofVec3f> nuevosVertices;
			nuevosVertices.push_back(pWorldA);
			nuevosVertices.push_back(nuevoVerticeB);
			nuevosVertices.push_back(nuevoVerticeC);
			nuevosVertices.push_back(nuevoVerticeD);			

			// Actualizar la mesa con los nuevos vértices
			table->getPolygonModelObject()->setVertexs(nuevosVertices);
			table->initOrderedVertexs = nuevosVertices;
		}
		/**************** Fin de la calibración inicial de la mesa ********************/
	}

	void compare(vector<ofVec3f>& initOrderedVertexs, vector<ofVec3f>& detectedVertexs, int& minDistanceIndex, 
			int& correspondingMinDistanceIndex) {
		// Comparar los nuevos vértices detectados con los vértices iniciales de la mesa
		vector<float> distance;
		distance.resize(4);
		vector<int> detectedVertexIndex;
		detectedVertexIndex.resize(4);

		float minDistance, currentDistance;
		for (int i = 0; i < initOrderedVertexs.size(); i++) 
		{
			minDistance = 100;
			ofVec3f currentInitVertex = initOrderedVertexs.at(i);
			for (int j = 0; j < detectedVertexs.size(); j++) 
			{
				currentDistance = currentInitVertex.distance(detectedVertexs.at(j));
				if ( currentDistance < minDistance && isTableVertexInSafeArea(detectedVertexs.at(j)) ) 
				{
					minDistance = currentDistance;
					distance.at(i) =  currentDistance;
					detectedVertexIndex.at(i) = j;
				}
			}
		}

		// Finalmente recorro la estructura para obtener la pareja de vértices con distancia mínima
		minDistance = 100;
		minDistanceIndex = -1;
		for (int i = 0; i < 4; i ++) {
			if (distance.at(i) < minDistance) {
				minDistance = distance.at(i);
				minDistanceIndex = i;
			}
		}

		correspondingMinDistanceIndex = detectedVertexIndex.at(minDistanceIndex);
	}


	TablePtr Table::updateTablePlane(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud)
	{
		TablePtr table(new Table(coefficients, cloud));

		vector<ofVec3f> tableModelVertexs(gModel->getTable()->getPolygonModelObject()->getMathModel().getVertexs());

		Plane3D newPlane(coefficients);
		Polygon* p = table->getPolygonModelObject();
		p->setPlane(newPlane);
//		p->setVertexs(tableModelVertexs);
		p->setCenter(computeCentroid(cloud));
		table->setDrawPointCloud(false);

		table->initOrderedVertexs = gModel->getTable()->initOrderedVertexs;

		// Re detectar el rectángulo de la mesa, para detectar de nuevo los vértices 
		vector<ofVec3f> detectedVertexs = findRectangle(cloud, coefficients);
		
		// Ordenar los vértices, en sentido horario
		detectedVertexs = reorderTableVertexs(detectedVertexs);

		int minDistanceIndex = -1;
		int correspondingMinDistanceIndex = -1;
		compare(table->initOrderedVertexs,detectedVertexs,minDistanceIndex,correspondingMinDistanceIndex);

		// Se debe prolongar la mesa segun sea necesario, para mantener la calibración inicial. Se toma como inicial la pareja de distancia mínima.
		if (minDistanceIndex == -1) {		// || minDistance > 0.30) {		// Si la distancia entre vértices es mayor a 30 cms
			cout << "No se pudo redeterminar los vértices" << endl;
			/* Actualizar vértices proyectándolos en el nuevo plano */
			for (int i = 0; i < tableModelVertexs.size(); i++)
				tableModelVertexs[i] = newPlane.project(tableModelVertexs[i]);
			// Actualizar la mesa con los vértices proyectados
			table->getPolygonModelObject()->setVertexs(tableModelVertexs);
		} else {
			vector<ofVec3f> newVertexs;	
			const int kVertexs = 4;
			newVertexs.resize(kVertexs);
			// La pareja de vértices más cercanos es initOrderedVertexs.at(minDistanceComparison) 
			//	y detectedVertexs.at(comparison->at(minDistanceComparison).detectedVertexIndex)
			for (int i = 0; i < kVertexs; i++) {
				int index = (minDistanceIndex + i) % kVertexs;
				newVertexs.at(index) =
					detectedVertexs.at((correspondingMinDistanceIndex + i) % kVertexs);
//				cout << "newVertexs.at(" << index << ") vale: (" << ofVecToString(newVertexs.at(index)) << endl;
			}

			ofVec3f startVertex = newVertexs.at(minDistanceIndex); 
			ofVec3f nextVertex, prevVertex;
			// Ajustar los vértices para que las distancias iniciales (de la calibración inicial) se mantengan
			int indexNext, indexPrevious, indexOpposite;
			double dist, calibratedDistance;
			// Ajustar siguiente vértice
				indexNext = minDistanceIndex + 1;
				if (indexNext >= newVertexs.size()) 
					indexNext = 0;				
				nextVertex = newVertexs.at(indexNext);
				mapinect::Line3D lineNext(startVertex, nextVertex);	
				dist = startVertex.distance(nextVertex);
//				cout << "Siguiente vertice - dist = " << dist << endl;
				calibratedDistance = table->initOrderedVertexs.at(minDistanceIndex).distance(table->initOrderedVertexs.at(indexNext)); 
//				cout << "  y calibratedDistance = " << calibratedDistance << endl;
				newVertexs.at(indexNext) = lineNext.calculateValue( calibratedDistance / dist);
//				cout << "newVertexs.at(" << indexNext << ") ahora vale: (" << ofVecToString(newVertexs.at(indexNext)) << endl;
			// Ajustar vértice anterior
				indexPrevious = minDistanceIndex - 1;
				if (indexPrevious < 0) 
					indexPrevious = newVertexs.size() - 1;
				prevVertex = newVertexs.at(indexPrevious);
				mapinect::Line3D linePrev(startVertex, prevVertex);	
				dist = startVertex.distance(prevVertex);
//				cout << "Vertice anterior - dist = " << dist << endl;
				calibratedDistance = table->initOrderedVertexs.at(minDistanceIndex).distance(table->initOrderedVertexs.at(indexPrevious)); 
//				cout << "  y calibratedDistance = " << calibratedDistance << endl;
				newVertexs.at(indexPrevious) = linePrev.calculateValue( calibratedDistance / dist);
//				cout << "newVertexs.at(" << indexPrevious << ") ahora vale: (" << ofVecToString(newVertexs.at(indexPrevious)) << endl;
			// Ajustar vértice opuesto, con suma de vectores
				indexOpposite = indexNext + 1;
				if (indexOpposite >= newVertexs.size()) 
					indexOpposite = 0;
				// (nextVertex - startVertex) + (prevVertex - startVertex) = (opVertex - startVertex)
				// => opVertex = (nextVertex - startVertex) + (prevVertex - startVertex) + startVertex
				newVertexs.at(indexOpposite) = newVertexs.at(indexNext) - newVertexs.at(minDistanceIndex) + newVertexs.at(indexPrevious);
//				cout << "newVertexs.at(" << indexOpposite << ") ahora vale: (" << ofVecToString(newVertexs.at(indexOpposite)) << endl;
			// Actualizar la mesa con los nuevos vértices
			Polygon* p = table->getPolygonModelObject();
			p->setVertexs(newVertexs);
		}

		return table;
//		gModel->setTable(table);
	}

	Eigen::Affine3f Table::calculateRotations(const pcl::ModelCoefficients& coefficients)
	{
		Eigen::Vector3f axisX(1, 0, 0);
		Eigen::Vector3f axisZ(0, 0, 1);
		ofVec3f ejeX = ofVec3f(1, 0, 0);
		ofVec3f ejeZ = ofVec3f(0, 0, 1);
		Eigen::Affine3f rotationX, rotationZ;
		ofVec3f normalMesa = ofVec3f(0, -1, 0); // Queremos que la mesa tenga siempre esta normal
		ofVec3f newTableNormal = (::getNormal(coefficients).getNormalized());		// La normal debe estar apuntando hacia Y negativo

		// Calcular rotaciones necesarias para que la nueva mesa tenga normal (0,-1,0)
		// Proyectar a la normal newTableNormal en el plano X=x; plano que pasa por el origen, con normal = (1,0,0) 
		Plane3D planoX = Plane3D(ofVec3f(0, 0, 0), ejeX); //lo defino con el plano que yz y normal x
		ofVec3f projXNormal = planoX.project(newTableNormal);
		// Calcular el ángulo que forman la proyección de la normal con el vector (0,-1,0)
		float angleX = projXNormal.angleRad(normalMesa);
		projXNormal.normalize();
		if (projXNormal.crossed(normalMesa).x < 0) {
			angleX *= -1;
		}
		rotationX = Eigen::AngleAxis<float>(angleX, axisX);

/*		if (IsFeatureMoveArmActive()) {		//TODO: verificar por qué hay que cambiar esta rotación, según si el feature move arm está activo o no
			rotationX = Eigen::AngleAxis<float>(angleX, axisX);		// Si el feature Move ARm está activo
		} else {
			rotationX = Eigen::AngleAxis<float>(-angleX, axisX);
		}
*/

		// Rotar a la normal newTableNormal en X el ángulo hallado
		ofVec3f normalRotatedX = transformPoint(newTableNormal,rotationX);
		// Calcular el ángulo que forman la normal rotada con el vector (0,-1,0)
		float angleZ = normalRotatedX.angleRad(normalMesa);
		normalRotatedX.normalize();
		if (normalRotatedX.crossed(normalMesa).z < 0) {
			angleZ *= -1;
		}
		rotationZ = Eigen::AngleAxis<float>(angleZ, axisZ);

		// Calcular la matriz de rotación de Eigen para aplicar estas dos rotaciones
		Eigen::Affine3f rotation = rotationZ * rotationX;

		ofVec3f checkTableNormal = transformPoint(newTableNormal, rotation);

		return rotation;
	}

	TablePtr Table::create(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud)
	{
		PCPtr transformedCloud(cloud);
		pcl::ModelCoefficients transformedCoefficients(coefficients);

		ofVec3f centroideMesa = computeCentroid(cloud); 
		cout << "centroide = " << centroideMesa << endl;


		// We want to set the table as the plane Y = y (not necesarilly at y=0, but needs to have normal = (0,-1,0))
		Eigen::Affine3f rotation = Table::calculateRotations(coefficients);
		Eigen::Affine3f composedMatrix, translation;

		// The table should be on this plane, y = Y
//		float Y = 0.38;
		float Y = 0;

		if (!IsFeatureMoveArmActive())
		{
			// While we don't have the arm transformation, we will translate the origin to
			// table's centroid directly from Kinect's origin
			ofVec3f centroid(computeCentroid(cloud));
			translation = Eigen::Translation<float, 3>(-centroid.x, -centroid.y, -centroid.z);

			composedMatrix = rotation * translation;		
			transformedCloud = PCPtr(transformCloud(cloud, composedMatrix));
		} else {
//			transformedCloud = PCPtr(transformCloud(cloud, rotation));
//			float translateY = Y - computeCentroid(transformedCloud).y;
/*			float translateY = 0;
			translation = Eigen::Translation<float, 3>(0.0, translateY, 0.0);
*/
			ofVec3f centroid(computeCentroid(cloud));
			
			composedMatrix = getTranslationMatrix(centroid) * (rotation * getTranslationMatrix(-centroid));

			transformedCloud = PCPtr(transformCloud(cloud, composedMatrix));

			ofVec3f newCentroid = computeCentroid(transformedCloud);
			cout << "centroide = " << newCentroid << endl;
			Y = newCentroid.y; 

			composedMatrix = composedMatrix * gTransformation->getWorldTransformation();
		}
		saveCloud("mesaInicial.pcd",*transformedCloud);

		transformedCoefficients.values[0] = 0;
		transformedCoefficients.values[1] = -1;
		transformedCoefficients.values[2] = 0;
		transformedCoefficients.values[3] = Y;

		gTransformation->setWorldTransformation(composedMatrix);
		gTransformation->setInitialWorldTransformation(composedMatrix);

		TablePtr table(new Table(transformedCoefficients, transformedCloud));
		table->detect();
		table->setDrawPointCloud(false);


		// reorder the vertexs so they're counter-clockwise and A(0) will be the closest to (0, 0) in screen coords
		table->getPolygonModelObject()->setVertexs(reorderTableVertexs(table->getPolygonModelObject()->getMathModel().getVertexs()));
		
		if (IsFeatureCalibrateTableActive())
		{
			calibrateTable(table);
		}

		table->initOrderedVertexs = table->getPolygonModelObject()->getMathModel().getVertexs();

		ofVec3f ab = table->initOrderedVertexs.at(1) - table->initOrderedVertexs.at(0);
		ab = ab.getNormalized();
		ofVec3f ad = table->initOrderedVertexs.at(3) - table->initOrderedVertexs.at(0);
		ad = ad.getNormalized();
		float angleDAB = ab.angle(ad);
		cout << "angle DAB es: " << angleDAB << endl;

		ofVec3f cd = table->initOrderedVertexs.at(3) - table->initOrderedVertexs.at(2);
		cd = cd.getNormalized();
		float angleADC = ad.angle(cd);
		cout << "angle ADC es: " << angleADC << endl;


		gModel->setTable(table);

		return table;
	}

	IObjectPtr Table::getMathModelApproximation() const
	{
		vector<IPolygonPtr> polygons;
		polygons.push_back(getMathPolygonModelApproximation());
		IObjectPtr result(new DataObject(getId(), getCenter(), getScale(), getRotation(), polygons));
		dynamic_cast<Polygon*>(polygons[0].get())->setContainer(result);
		return result;
	}

	bool Table::isOnTable(const PCPtr& cloud)
	{
		//Busco el mayor y
		int indexMax = -1;
		float max = -1;
		for(int i = 0; i < cloud->points.size(); i++)
		{
			if(cloud->points.at(i).y > max)
			{
				indexMax = i;
				max = cloud->points.at(i).y;
			}
		}

		if (indexMax >= 0)
		{
			return abs(evaluatePoint(getCoefficients(), PCXYZ_OFVEC3F(cloud->points.at(indexMax))))
				< Constants::TABLE_HEIGHT_TOLERANCE() * 2.0f;
		}
		else
		{
			return false;
		}
	}

	bool Table::isParallelToTable(const PCPolygonPtr& polygon)
	{
		ofVec3f polygonNormal = polygon->getNormal();
		float dot = abs(getNormal().dot(polygonNormal));
		return dot > 0.9;
	}

	bool Table::isOverTable(const PCPtr& cloud)
	{
		Polygon3D pol = this->getMathPolygonModelApproximation()->getMathModel();

		for(int i = 0; i < cloud->size(); i++)
		{
			if (pol.isInPolygon(pol.getPlane().project(PCXYZ_OFVEC3F(cloud->at(i)))))
				return true;
		}
		return false;

		ofVec3f tableNorm = this->getNormal();
	
		ofVec3f minV, maxV;
		minV = this->getvMin();
		maxV = this->getvMax();

		maxV += tableNorm;
	
		/*createCloud(maxV,"maxt.pcd");
		createCloud(minV,"mint.pcd");
		pcl::io::savePCDFile("table.pcd",*table->getCloud());
		pcl::io::savePCDFile("hand.pcd",*cloud);*/
		//Alcanza con que un punto esté sobre la mesa
	
		for(int i = 0; i < cloud->size(); i++)
		{
			pcl::PointXYZ pto = cloud->at(i);
			if((pto.x > min(minV.x,maxV.x) && pto.x < max(maxV.x,minV.x)) &&
			   (pto.y > min(minV.y,maxV.y) && pto.y < max(maxV.y,minV.y)) &&
			   (pto.z > min(minV.z,maxV.z) && pto.z < max(maxV.z,minV.z)))
			   return true;
		}
	
		return false;

	}
}
