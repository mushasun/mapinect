#include "Table.h"

#include "Constants.h"
#include "DataObject.h"
#include "Feature.h"
#include "Globals.h"
#include "mapinectTypes.h"
#include "pointUtils.h"

namespace mapinect
{
	TablePtr Table::Create(const pcl::ModelCoefficients& coefficients, const PCPtr& cloud)
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
		gModel->setTable(table);

		if (IsFeatureCalibrateTableActive())
		{
			/************************************************************************/
			/*     Calibración inicial de la Mesa - Tamaño de la mesa en el XML  	*/
			vector<ofVec3f> vertexs  = gModel->getTable()->getPolygonModelObject()->getMathModel().getVertexs();
			// Los vértices del Math Model vienen en orden, pero no se sabe cuál es el primero 
			//				   A *------------* A+1
			//					/			   \
			//				   /			    \
			//			  A+3 *------------------* A+2

			int TOLERANCE = Constants::PIXEL_TOLERANCE_ESTIMATED_VERTEX;
			float anchoMesa = Constants::TABLE_WIDTH;
			float largoMesa = Constants::TABLE_LENGTH;

			float minX = 0 + TOLERANCE;
			float maxX = KINECT_DEFAULT_WIDTH - TOLERANCE;
			float minY = 0 + TOLERANCE;
			float maxY = KINECT_DEFAULT_HEIGHT - TOLERANCE;
			ofPoint pScreenA = ofPoint(KINECT_DEFAULT_WIDTH,KINECT_DEFAULT_HEIGHT,0); // En coordenadas de pantalla
			ofPoint pWorldA = ofPoint(0,0,0);
			int indexA = -1;
			// Verificar si hay vértices estimados
			// Buscar cuál es el vértice A
			//	Será el que tenga máximo Z y mínimo X
			for(vector<ofVec3f>::const_iterator it = vertexs.begin(); it != vertexs.end(); it++) 
			{
				ofPoint p = getScreenCoords(*it);
				// Si está fuera de pantalla, es porque el Kinect no llega a ver a ese punto
				//	y por lo tanto fue estimado en la detección del rectángulo de la mesa
				// Si está dentro, pero sobre el borde, es un punto estimado (la mesa se corta)
				if(inRange(p.x, minX, maxX) && inRange(p.y, minY, maxY) && p.z > pScreenA.z && p.x < pScreenA.x)
				{
					// Si es un vértice real, visto, que no es estimado, y es el que tiene X mínima 
					pScreenA = p;
					pWorldA = *it;
					indexA = indexOf(vertexs,*it);
				}
			}

			if (pScreenA == ofPoint(KINECT_DEFAULT_WIDTH,KINECT_DEFAULT_HEIGHT,0)) 
			{
				cout << "Todos los vértices fueron estimados" << endl;
				cout << "Intente posicionar el brazo mirando hacia el vértice izquierdo de la mesa" << endl;
			} else {
				// maxZWorld es el vértice izquierdo más alejado de la cámara, y no es un vértice estimado
				ofVec3f v3A(vertexs.at(indexA));
			
				// Busco el siguiente (B), el anterior (D) y el opuesto (C) al punto A (con A = maxZ)
				int indexB = -1;
				if (indexA == vertexs.size() - 1) 
				{
					// Si es el último, el siguiente es el primero
					indexB = 0;
				} else {
					indexB = indexA + 1;
				}
				ofVec3f v3B(vertexs.at(indexB));

				int indexD = -1;
				if (indexA == 0) 
				{
					// Si es el primero, el anterior es el último
					indexD = vertexs.size() - 1;
				} else {
					indexD = indexA - 1;
				}
				ofVec3f v3D(vertexs.at(indexD));

				int indexC = -1;
				if (indexB == vertexs.size() - 1) 
				{
					// Si es el último, el siguiente es el primero
					indexC = 0;
				} else {
					indexC = indexB + 1;
				}
				ofVec3f v3C(vertexs.at(indexC));

				//       A -------- B ---------B'
				//	    /		      \
				//     /			   \
				//    D-----------------C
				//   /
				//  D'
				// anchoMesa = dist (A,B')
				// largoMesa = dist(A,D')

				// Verificar si B es estimado
				ofPoint pB = getScreenCoords(vertexs.at(indexB));
				ofVec3f nuevoVerticeB = vertexs.at(indexB);
				ofVec3f nuevoVerticeC = vertexs.at(indexC);
				// Si está fuera de pantalla, es porque el Kinect no llega a ver a ese punto
				//	y por lo tanto fue estimado en la detección del rectángulo de la mesa
				// Si está dentro, pero sobre el borde, es un punto estimado (la mesa se corta)
				if(!inRange(pB.x, minX, maxX) || !inRange(pB.y, minY, maxY))
				{
					// B fue estimado, entonces se debe calcular el nuevo B' con el ancho de mesa 
					mapinect::Line3D lineAB(v3A, v3B);
					double distanceAB = v3A.distance(v3B);
					nuevoVerticeB = lineAB.calculateValue(anchoMesa / distanceAB);
				}

				// Verificar si D es estimado
				ofPoint pD = getScreenCoords(vertexs.at(indexD));
				ofVec3f nuevoVerticeD = vertexs.at(indexD);
				// Si está fuera de pantalla, es porque el Kinect no llega a ver a ese punto
				//	y por lo tanto fue estimado en la detección del rectángulo de la mesa
				// Si está dentro, pero sobre el borde, es un punto estimado (la mesa se corta)
				if(!inRange(pD.x, minX, maxX) || !inRange(pD.y, minY, maxY))
				{
					// D fue estimado, entonces se debe calcular el nuevo D' con el largo de mesa 
					mapinect::Line3D lineAD(v3A,v3D);
					double distanceAD = v3A.distance(v3D);
					nuevoVerticeD = lineAD.calculateValue(largoMesa / distanceAD);
				}

				if (nuevoVerticeB != vertexs.at(indexB) || nuevoVerticeD != vertexs.at(indexD)) 
				{
					// Si B o D fueron recalculados, se debe actualizar C también
					// AB + AD = AC
					// AC = C - A
					// => C = (AB + AD) + A = (B-A + D-A) + A
					ofVec3f AC = nuevoVerticeB - v3A + nuevoVerticeD - v3A; 
					nuevoVerticeC = AC + v3A;

					vector<ofVec3f> nuevosVertices;
					nuevosVertices.push_back(v3A);
					nuevosVertices.push_back(nuevoVerticeB);
					nuevosVertices.push_back(nuevoVerticeC);
					nuevosVertices.push_back(nuevoVerticeD);			

					// Actualizar la mesa con los nuevos vértices
					gModel->getTable()->getPolygonModelObject()->setVertexs(nuevosVertices);
				}

			}
			/**************** Fin de la calibración inicial de la mesa ********************/
		}

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
