#include "Table.h"

#include "Constants.h"
#include "DataObject.h"
#include "Feature.h"
#include "Globals.h"
#include "mapinectTypes.h"
#include "pointUtils.h"
#include <algorithm>
#include "SortPolar.h"
#include <cmath>

namespace mapinect
{
	vector<ofVec3f> reorderTableVertexs(const vector<ofVec3f>& vertexs)
	{
		// Se deben ordenar los vértices en screen coords
		//	de modo que el A sea el que está mas cerca del (0,0) en Screen coords
		//	y luego en sentido anti-horario B, C y D
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
			// We want orderedVertexs2D[0] = vertexs2D[indexAVertex2D] and keep the ordering;
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

		return orderedVertexs3D;
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

	void calibrateTable(TablePtr& table)
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

		//       A -------- D -----D'
		//	    /		     \
		//     /			  \
		//    B----------------C
		//   /
		//  B'
		// TABLE_LENGTH_AB = dist(A,B')
		// TABLE_LENGTH_AD = dist(A,D')

		mapinect::Line3D lineAB(pWorldA, pWorldB);	// Linea entre A y B, ambos en 3D
		double distanceAB = pWorldA.distance(pWorldB);
		// Se va a modificar el vertice B solo si fue estimado, o si se quiere acortar el lado AB
		if(!isTableVertexInSafeArea(pWorldB) || (TABLE_LENGTH_AB < distanceAB)) {
			// B fue estimado, entonces se debe calcular el nuevo B' con el ancho de mesa 
			nuevoVerticeB = lineAB.calculateValue(TABLE_LENGTH_AB / distanceAB);
			cout << "El vertice B fue modificado" << endl;
		}

		mapinect::Line3D lineAD(pWorldA,pWorldD);	// Linea entre A y D, ambos en 3D
		double distanceAD = pWorldA.distance(pWorldD);
		// Se va a modificar el vertice D solo si fue estimado, o si se quiere acortar el lado AD
		if(!isTableVertexInSafeArea(pWorldD) || (TABLE_LENGTH_AD < distanceAD)) {
			// D fue estimado, entonces se debe calcular el nuevo D' con el largo de mesa 
			nuevoVerticeD = lineAD.calculateValue(TABLE_LENGTH_AD / distanceAD);
			cout << "El vertice D fue modificado" << endl;
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
		}
		/**************** Fin de la calibración inicial de la mesa ********************/
	}

	void Table::updateTablePlane(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud)
	{
		TablePtr table(new Table(coefficients, cloud));

		vector<ofVec3f> vertexs(gModel->getTable()->getPolygonModelObject()->getMathModel().getVertexs());

		Plane3D newPlane(coefficients);
		table->getPolygonModelObject()->setPlane(newPlane);
		table->setDrawPointCloud(false);

		/* Update vertexs - projecting on new plane */
		for (int i = 0; i < vertexs.size(); i++)
			vertexs[i] = newPlane.project(vertexs[i]);

		table->getPolygonModelObject()->setVertexs(vertexs);

		gModel->setTable(table);
	}

	TablePtr Table::create(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud)
	{
		PCPtr transformedCloud(cloud);
		pcl::ModelCoefficients transformedCoefficients(coefficients);
		
		if (!IsFeatureMoveArmActive())
		{
			// We want to set the table as the plane y = 0
			// While we don't have the arm transformation, we will translate the origin to
			// table's centroid directly from Kinect's origin
			Eigen::Vector3f axisX(1, 0, 0);
			Eigen::Affine3f rotationX;
			ofVec3f newYaxis = -(::getNormal(coefficients).getNormalized());
			float angle = newYaxis.angleRad(ofVec3f(0, 1, 0));
			rotationX = Eigen::AngleAxis<float>(-angle, axisX);

			Eigen::Affine3f translation;
			ofVec3f centroid(computeCentroid(cloud));
			translation = Eigen::Translation<float, 3>(-centroid.x, -centroid.y, -centroid.z);

			Eigen::Affine3f composedMatrix = rotationX * translation;
		
			gTransformation->setWorldTransformation(composedMatrix);
			gTransformation->setInitialWorldTransformation(composedMatrix);

			transformedCloud = PCPtr(transformCloud(cloud, composedMatrix));

			transformedCoefficients.values[0] = 0;
			transformedCoefficients.values[1] = -1;
			transformedCoefficients.values[2] = 0;
			transformedCoefficients.values[3] = 0;
		}
		
		TablePtr table(new Table(transformedCoefficients, transformedCloud));
		table->detect();
		table->setDrawPointCloud(false);

		// reorder the vertexs so they're counter-clockwise and A(0) will be the closest to (0, 0) in screen coords
		table->getPolygonModelObject()->setVertexs(reorderTableVertexs(table->getPolygonModelObject()->getMathModel().getVertexs()));
		
		if (IsFeatureCalibrateTableActive())
		{
			calibrateTable(table);
		}

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
		int idx_max = -1;
		float max = -1;
		for(int i = 0; i < cloud->points.size(); i++)
		{
			if(cloud->points.at(i).y > max)
			{
				idx_max = i;
				max = cloud->points.at(i).y;
			}
		}

		if (idx_max >= 0)
		{
			return abs(evaluatePoint(getCoefficients(), PCXYZ_OFVEC3F(cloud->points.at(idx_max))))
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
