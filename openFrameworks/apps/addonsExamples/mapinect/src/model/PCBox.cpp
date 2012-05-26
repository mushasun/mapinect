#include "PCBox.h"

#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>

#include "ofUtils.h"

#include "Constants.h"
#include "Globals.h"
#include "ofVecUtils.h"
#include "PCQuadrilateral.h"
#include "pointUtils.h"
#include "utils.h"


#define MAX_FACES		3

namespace mapinect {

	PCBox::PCBox(const PCPtr& cloud, int objId)
				: PCPolyhedron(cloud, objId)
	{
	}

	void PCBox::detectPrimitives() {
		PCPolyhedron::detectPrimitives();
		if(pcpolygons.size() == 6)
			partialEstimation = true;
		messureBox();
	}

	const PCPolygonPtr PCBox::getPCPolygon(IPolygonName name, const vector<PCPolygonPtr>& newPolygons)
	{
		for(int i = 0; i < newPolygons.size(); i++)
			if(newPolygons.at(i)->getPolygonModelObject()->getName() == name)
				return newPolygons.at(i);

		return PCPolygonPtr();
	}

	void PCBox::unifyVertexs() {
		PCPolyhedron::unifyVertexs();
		return;
		vector<ofVec3f> unified;
		IPolygonName vertexNames[8][3]	= {{kPolygonNameSideA,kPolygonNameSideB,kPolygonNameTop},
									      {kPolygonNameSideA,kPolygonNameSideB,kPolygonNameBottom},
										  {kPolygonNameSideA,kPolygonNameSideD,kPolygonNameBottom},
										  {kPolygonNameSideA,kPolygonNameSideD,kPolygonNameTop},
										  {kPolygonNameSideC,kPolygonNameSideB,kPolygonNameTop},
									      {kPolygonNameSideC,kPolygonNameSideB,kPolygonNameBottom},
										  {kPolygonNameSideC,kPolygonNameSideD,kPolygonNameBottom},
										  {kPolygonNameSideC,kPolygonNameSideD,kPolygonNameTop}};
		int vertexIdx[8][3]				= {	{0,3,0},
											{1,2,0},
											{2,1,0},
											{3,0,0},
											{3,0,0},
											{2,1,0},
											{1,2,0},
											{0,3,0}};
		for(int i = 0; i < 8; i++)
		{
			PCPolygonPtr p0 = getPCPolygon(vertexNames[i][0], pcpolygons);
			PCPolygonPtr p1 = getPCPolygon(vertexNames[i][1], pcpolygons);
			PCPolygonPtr p2 = getPCPolygon(vertexNames[i][2], pcpolygons);

			if( p0 != NULL &&
				p1 != NULL &&
				p2 != NULL)
			{
				ofVec3f vertex = planeIntersection(p0->getCoefficients(),p1->getCoefficients(),p2->getCoefficients());
				p0->getPolygonModelObject()->setVertex(vertexIdx[i][0],vertex);
				p1->getPolygonModelObject()->setVertex(vertexIdx[i][1],vertex);
				p2->getPolygonModelObject()->setVertex(vertexIdx[i][2],vertex);
				
				unified.push_back(vertex);
				//DEBUG
				createCloud(vertex,"vmerged" + ofToString(i) + ".pcd");
				saveCloudAsFile("merged" + ofToString(i) + "1.pcd",*p0->getCloud());
				saveCloudAsFile("merged" + ofToString(i) + "2.pcd",*p1->getCloud());
				saveCloudAsFile("merged" + ofToString(i) + "3.pcd",*p2->getCloud());

			}

			

			

		}
		
		createCloud(unified,"UnifiedVertex.pcd");

		//for(int i = 0; i < vertexs.size(); i++)
		//{
		//	cout << "vertex " << ofToString(i) << ": " << vertexs.at(i).getPoint() << endl;
		//	for(int j = 0; j < vertexs.at(i).Polygons.size(); j ++)
		//		cout << "\t" << vertexs.at(i).Polygons.at(j)->getPolygonModelObject()->getName() << endl;
		//}
	}
	
	vector<PCPolygonPtr> PCBox::discardPolygonsOutOfBox(const vector<PCPolygonPtr>& toDiscard)
	{
		vector<PCPolygonPtr> polygonsInBox;
		TablePtr table = gModel->table;

		//pcl::io::savePCDFile("table.pcd",table->getCloud());
		
		for(int i = 0; i < toDiscard.size(); i++)
		{
			//pcl::io::savePCDFile("pol" + ofToString(i) + ".pcd",toDiscard.at(i)->getCloud());

			PCPtr cloudPtr(toDiscard.at(i)->getCloud());
			if(table->isOnTable(cloudPtr))
			{
				cout << "pol" << ofToString(i) << " On table!" <<endl;
				polygonsInBox.push_back(toDiscard.at(i));
			}
			else if(table->isParallelToTable(toDiscard.at(i)))
			{
				cout << "pol" << ofToString(i) << " parallel table!" <<endl;
				polygonsInBox.push_back(toDiscard.at(i));
				//pcl::io::savePCDFile("pol" + ofToString(i) + ".pcd",toDiscard.at(i)->getCloud());
			}

		}

		return polygonsInBox;
	}
	
	PCPolygonPtr PCBox::duplicatePol(const PCPolygonPtr& polygon,const vector<PCPolygonPtr>& newPolygons)
	{
		PCPolygonPtr f1;

		//Encuentro el poligono 'vecino' al que quiero duplicar
		TablePtr t = gModel->table;
		bool parallelAndNotInContact = t->isParallelToTable(polygon) && !t->isOnTable(polygon->getCloud());
		bool foundFace = false;
		//saveCloudAsFile ("from.pcd", *polygon->getCloud());
		/// Si es paralelo a la mesa (cara techo), tomo cualquier otro poligono
		/// Si es perpendicular, busco otro poligono perpendicular
		for(int i = 0; i < newPolygons.size(); i ++)
		{
			if(newPolygons.at(i) != polygon)
			{
				if((!parallelAndNotInContact && (!t->isParallelToTable(newPolygons.at(i)) || t->isOnTable(newPolygons.at(i)->getCloud()))) ||
					parallelAndNotInContact)
				{
					f1 = newPolygons.at(i);
					foundFace = true;
					break;
				}
			}
		}
		/// Si no encontre otro poligono que cumple las condiciones anteriores, tomo cualquier poligono 
		/// que no sea el mismo
		if(!foundFace)
		{
			for(int i = 0; i < newPolygons.size(); i ++)
			{
				if(newPolygons.at(i) != polygon)
				{
					f1 = newPolygons.at(i);
					break;
				}
			}
		}

		//Busco el largo que tengo que desplazar
		vector<ofVec3f> vex = f1->getPolygonModelObject()->getVertexs();
		
		if(!parallelAndNotInContact && foundFace) // Busco los 2 puntos con menor 'y' para hallar el ancho de la cara
		{
			sort(vex.begin(),vex.end(),yAxisSortDes);
		}
		else // Busco los 2 puntos con menor 'x' para hallar el alto de la cara
		{
			sort(vex.begin(),vex.end(),xAxisSortDes);
		}
		ofVec3f min1 = vex.at(0);
		ofVec3f min2 = vex.at(1);
		

		/*createCloud(min1, "min1.pcd");
		createCloud(min2, "min2.pcd");*/


		float f1Width = abs((min1 - min2).length());

		PCPtr cloudToMove = polygon->getCloud();
		PCPtr cloudMoved (new PC()); 
		ofVec3f normal = polygon->getNormal();
		//pcl::flipNormalTowardsViewpoint(cloudToMove->at(0),0,0,0,normal.x,normal.y,normal.z);
		normal *= -f1Width; // TODO: Corregir dirección de la normal
		Eigen::Transform<float,3,Eigen::Affine> transformation;
		transformation = Eigen::Translation<float,3>(normal.x, normal.y, normal.z);

		pcl::transformPointCloud(*cloudToMove,*cloudMoved,transformation);

		//Corrección de coeficiente
		pcl::ModelCoefficients coeff = polygon->getCoefficients();
		
		//invierto la normal
		coeff.values[0] *= -1;
		coeff.values[1] *= -1;
		coeff.values[2] *= -1;
		//TODO: Corregir parametro 'd' de la ecuacion del plano

		PCPolygonPtr estimatedPol (new PCQuadrilateral(coeff,cloudMoved,f1->getId()*(-2),true));
		estimatedPol->detectPolygon();
		estimatedPol->getPolygonModelObject()->setContainer(this);
		
		IPolygonName polFatherName = polygon->getPolygonModelObject()->getName();
		IPolygonName polName;

		if(polFatherName == kPolygonNameTop)
			polName = kPolygonNameBottom;
		else if(polFatherName == kPolygonNameBottom)
			polName = kPolygonNameTop;
		else if(polFatherName <= 2)
			polName = (IPolygonName)(polFatherName + 2);
		else
			polName = (IPolygonName)(polFatherName - 2);

		estimatedPol->getPolygonModelObject()->setName(polName);

		/*
		saveCloudAsFile ("estimated.pcd", *estimatedPol->getCloud());
		saveCloudAsFile ("from.pcd", *f1->getCloud());
		saveCloudAsFile ("original.pcd", *cloudToMove);*/


		return estimatedPol;
	}

	list<IPolygonName> PCBox::getMissing(const vector<PCPolygonPtr>& estimated)
	{
		list<IPolygonName> missing;
		missing.push_back(kPolygonNameTop);
		missing.push_back(kPolygonNameSideA);
		missing.push_back(kPolygonNameSideB);
		missing.push_back(kPolygonNameSideC);
		missing.push_back(kPolygonNameSideD);
		missing.push_back(kPolygonNameBottom);

		for(int i = 0; i < estimated.size(); i ++)
		{
			IPolygonName searched = estimated.at(i)->getPolygonModelObject()->getName();
			missing.remove(searched);
		}

		return missing;
	}

	PCPolygonPtr PCBox::getNextPolygon(IPolygonName toEstimate, const vector<PCPolygonPtr>& newPolygons)
	{
		int next = (toEstimate + 1) % 6;
		if(next == 5)
			next = 1;

		IPolygonName searchedPolygon = (IPolygonName) next;

		for(int i = 0; i < newPolygons.size(); i++)
			if(newPolygons.at(i)->getPolygonModelObject()->getName() == searchedPolygon)
				return newPolygons.at(i);
		
		throw::exception("No Polygon");
	}

	PCPolygonPtr PCBox::getPrevPolygon(IPolygonName toEstimate, const vector<PCPolygonPtr>& newPolygons)
	{
		int prev = (toEstimate - 1) % 6;
		if(prev == 0)
			prev = 4;

		IPolygonName searchedPolygon = (IPolygonName) prev;

		for(int i = 0; i < newPolygons.size(); i++)
			if(newPolygons.at(i)->getPolygonModelObject()->getName() == searchedPolygon)
				return newPolygons.at(i);
		
		throw::exception("No Polygon");
	}

	PCPolygonPtr PCBox::getOppositePolygon(IPolygonName toEstimate, const vector<PCPolygonPtr>& newPolygons)
	{
		int next = (toEstimate + 2) % 6;
		if(next == 5)
			next = 1;
		if(next == 0)
			next = 2;

		IPolygonName searchedPolygon = (IPolygonName) next;

		for(int i = 0; i < newPolygons.size(); i++)
			if(newPolygons.at(i)->getPolygonModelObject()->getName() == searchedPolygon)
				return newPolygons.at(i);
		
		throw::exception("No Polygon");
		;
	}
	

	vector<PCPolygonPtr> PCBox::estimateHiddenPolygons(const vector<PCPolygonPtr>& newPolygons)
	{
		vector<PCPolygonPtr> estimated;
		if(newPolygons.size() <= 1)
			return estimated;

		for(int i = 0; i < newPolygons.size(); i ++)
		{
			PCPolygonPtr estimatedPol = duplicatePol(newPolygons.at(i),newPolygons);
			estimated.push_back(estimatedPol);
			saveCloudAsFile ("estimated" + ofToString(estimatedPol->getPolygonModelObject()->getName()) + ".pcd", *estimatedPol->getCloud());
		}
		
		vector<PCPolygonPtr> partialEstimation = estimated;
		partialEstimation.insert(partialEstimation.begin(), newPolygons.begin(), newPolygons.end());

		if(partialEstimation.size() < 6)
		{
			
			int maxIter = 1;
			int i = 0;
			

			while(partialEstimation.size() < 6 && 
				  maxIter > 0 &&
				  partialEstimation.size() > 1)
			{
				list<IPolygonName> missing = getMissing(partialEstimation);
				maxIter--;
				IPolygonName toEstimate = *missing.begin();
				PCPolygonPtr next, prev;
				vector<ofVec3f> nextVec, prevVec;

				if(toEstimate == kPolygonNameTop || toEstimate == kPolygonNameBottom)
				{
					next = partialEstimation.at(0);
					prev = getOppositePolygon(next->getPolygonModelObject()->getName(),partialEstimation);//Estimate TOP

					nextVec = next->getPolygonModelObject()->getVertexs();
					prevVec = prev->getPolygonModelObject()->getVertexs();

					sort(nextVec.begin(),nextVec.end(), yAxisSortAsc);
					sort(prevVec.begin(),prevVec.end(), yAxisSortAsc);
				}
				else
				{
					next = getNextPolygon(toEstimate,partialEstimation);
					prev = getPrevPolygon(toEstimate,partialEstimation);

					nextVec = next->getPolygonModelObject()->getVertexs();
					prevVec = prev->getPolygonModelObject()->getVertexs();

					bool ascending = toEstimate < 3;
					if(ascending)
					{
						sort(nextVec.begin(),nextVec.end(), xAxisSortAsc);
						sort(prevVec.begin(),prevVec.end(), xAxisSortAsc);
					}
					else
					{
						sort(nextVec.begin(),nextVec.end(), xAxisSortDes);
						sort(prevVec.begin(),prevVec.end(), xAxisSortDes);
					}
				}

				vector<ofVec3f> faceVertex;

				faceVertex.insert(faceVertex.end(),nextVec.begin(), nextVec.begin() + 2);
				faceVertex.insert(faceVertex.end(),prevVec.begin(), prevVec.begin() + 2);

				pcl::ModelCoefficients coef = findPlane(faceVertex.at(0),faceVertex.at(1),faceVertex.at(2));
				//fix normal
				ofVec3f internalPoint = nextVec.at(3);
				ofVec3f normal = ofVec3f(coef.values.at(0),coef.values.at(1),coef.values.at(2));
				float dist = normal.dot(internalPoint - faceVertex.at(0));
				if(dist > 0)
				{
					normal *= -1;
					coef.values.at(0) = normal.x;
					coef.values.at(1) = normal.y;
					coef.values.at(2) = normal.z;
				}

				PCPtr faceCloud (new PC());
				for(int i = 0; i < faceVertex.size(); i ++)
				{
					faceCloud->push_back(OFXVEC3F_POINTXYZ(faceVertex.at(i)));
				}

				PCPolygonPtr pcp(new PCQuadrilateral(coef, faceCloud, 99, true));
				pcp->detectPolygon();
				pcp->getPolygonModelObject()->setContainer(this);
				pcp->getPolygonModelObject()->setName(toEstimate);

					
				partialEstimation.push_back(pcp);
				estimated.push_back(pcp);

				PCPolygonPtr estimatedPol = duplicatePol(pcp,partialEstimation);
				partialEstimation.push_back(estimatedPol);
				estimated.push_back(estimatedPol);
				//createCloud(faceVertex, "estimatedFace.pcd");

				i++;
			}
		}
		return estimated;
	}

	void PCBox::addToModel(const PCPtr& nuCloud)
	{
		PCPolyhedron::addToModel(nuCloud);
		messureBox();
	}

	void PCBox::messureBox()
	{
		;
	}

	
}
