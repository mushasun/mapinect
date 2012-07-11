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

	//// Overrides
	// ------------------------------------------------------------------------------
	vector<PCPolygonPtr> PCBox::mergePolygons(vector<PCPolygonPtr>& toMerge)
	{
		vector<PCPolygonPtr> merged = PCPolyhedron::mergePolygons(toMerge);
		
		// Elimina caras opuestas y controla que no se agregue 2 veces la misma cara. 
		// No se pueden estar viendo caras que son opuestas en una misma escena
		vector<IPolygonName> hidden;
		vector<IPolygonName> added;
		vector<PCPolygonPtr> cleanedMerged;
		for(vector<PCPolygonPtr>::iterator iter = merged.begin(); iter != merged.end(); ++iter)
		{
			IPolygonName name = (*iter)->getPolygonModelObject()->getName();
			if(find(hidden.begin(), hidden.end(), name) == hidden.end())		// ya se agrego el opuesto?
			{
				if(find(added.begin(), added.end(), name) == added.end())		// ya se agrego esta cara?
				{
					cleanedMerged.push_back(*iter);
					added.push_back(name);
					hidden.push_back(getOppositePolygonName(name));				// no permitir agregar el opuesto
				}
			}
		}
		
		return cleanedMerged;
	}
	
	// ------------------------------------------------------------------------------
	void PCBox::unifyVertexs() {

		if(getMissing(pcpolygons).size() > 0)
		{
			cout << "FAILED ESTIMATION - OBJ: " << getId() << endl;//PCPolyhedron::unifyVertexs();
			for(int i = 0; i < pcpolygons.size(); i++)
				cout << "	" << pcpolygons.at(i)->getPolygonModelObject()->getName() << endl;
		}
		else
		{
			vertexs.clear();
			IPolygonName vertexNames[8][3]	= {{kPolygonNameSideA,kPolygonNameSideB,kPolygonNameTop},
											  {kPolygonNameSideA,kPolygonNameSideB,kPolygonNameBottom},
											  {kPolygonNameSideA,kPolygonNameSideD,kPolygonNameBottom},
											  {kPolygonNameSideA,kPolygonNameSideD,kPolygonNameTop},
											  {kPolygonNameSideC,kPolygonNameSideB,kPolygonNameTop},
											  {kPolygonNameSideC,kPolygonNameSideB,kPolygonNameBottom},
											  {kPolygonNameSideC,kPolygonNameSideD,kPolygonNameBottom},
											  {kPolygonNameSideC,kPolygonNameSideD,kPolygonNameTop}};
			int vertexIdx[8][3]				= {	{0,3,1},
												{1,2,2},
												{2,1,1},
												{3,0,2},
												{3,0,0},
												{2,1,3},
												{1,2,0},
												{0,3,3}};
			vector<ofVec3f> tempVertexs [6];
			for(int j = 0; j < 6; j++)
			{
				tempVertexs[j].resize(4);
			}

			for(int i = 0; i < 8; i++)
			{
				PCPolygonPtr p0 = getPCPolygon(vertexNames[i][0], pcpolygons);
				PCPolygonPtr p1 = getPCPolygon(vertexNames[i][1], pcpolygons);
				PCPolygonPtr p2 = getPCPolygon(vertexNames[i][2], pcpolygons);

				if( p0.get() != NULL &&
					p1.get() != NULL &&
					p2.get() != NULL)
				{
					Plane3D plane0(p0->getPolygonModelObject()->getMathModel().getPlane());
					Plane3D plane1(p1->getPolygonModelObject()->getMathModel().getPlane());
					Plane3D plane2(p2->getPolygonModelObject()->getMathModel().getPlane());
					ofVec3f vertex = plane0.intersection(plane1, plane2);

					tempVertexs[vertexNames[i][0]][vertexIdx[i][0]] = vertex;
					tempVertexs[vertexNames[i][1]][vertexIdx[i][1]] = vertex;
					tempVertexs[vertexNames[i][2]][vertexIdx[i][2]] = vertex;

					vertexs.push_back(vertex);
					//DEBUG
					/*saveCloudAsFile("vmerged" + ofToString(i) + ".pcd", vertex);
					saveCloudAsFile("merged" + ofToString(i) + "1.pcd", *p0->getCloud());
					saveCloudAsFile("merged" + ofToString(i) + "2.pcd", *p1->getCloud());
					saveCloudAsFile("merged" + ofToString(i) + "3.pcd", *p2->getCloud());*/

				}
			}
		
			for(int i = 0; i < 6; i++)
			{
				IPolygonName name = (IPolygonName)i;
				PCPolygonPtr p0 = getPCPolygon(name, pcpolygons);
				if(p0.get() != NULL)
					p0->getPolygonModelObject()->setVertexs(tempVertexs[i]);
			}
			saveCloud("UnifiedVertex.pcd", vertexs);

			/*for(int i = 0; i < pcpolygons.size(); i ++)
			{
				saveCloudAsFile("p" + ofToString(pcpolygons.at(i)->getPolygonModelObject()->getName()) + ".pcd", *pcpolygons.at(i)->getCloud());
				saveCloudAsFile("v" + ofToString(pcpolygons.at(i)->getPolygonModelObject()->getName()) + ".pcd", pcpolygons.at(i)->getPolygonModelObject()->getMathModel().getVertexs());
			}*/
			//for(int i = 0; i < vertexs.size(); i++)
			//{
			//	cout << "vertex " << ofToString(i) << ": " << vertexs.at(i).getPoint() << endl;
			//	for(int j = 0; j < vertexs.at(i).Polygons.size(); j ++)
			//		cout << "\t" << vertexs.at(i).Polygons.at(j)->getPolygonModelObject()->getName() << endl;
			//}
		}
	}

	// ------------------------------------------------------------------------------
	vector<PCPolygonPtr> PCBox::estimateHiddenPolygons(const vector<PCPolygonPtr>& newPolygons)
	{
		vector<PCPolygonPtr> estimated;
		if(newPolygons.size() <= 0)
			return estimated;

		for(int i = 0; i < newPolygons.size(); i ++)
		{
			PCPolygonPtr estimatedPol = duplicatePol(newPolygons.at(i),newPolygons);
			estimated.push_back(estimatedPol);
			saveCloud("estimated" + ofToString(estimatedPol->getPolygonModelObject()->getName()) + ".pcd", *estimatedPol->getCloud());
		}
		
		vector<PCPolygonPtr> partialEstimation = estimated;
		partialEstimation.insert(partialEstimation.begin(), newPolygons.begin(), newPolygons.end());
	
		int maxIter = 3;
		int i = 0;
			
		list<IPolygonName> missing = getMissing(partialEstimation);

		while(partialEstimation.size() < 6 && 
				maxIter > 0 &&
				partialEstimation.size() > 1 &&
				missing.size() > 0)
		{
			maxIter--;
			IPolygonName toEstimate = *missing.begin();
			PCPolygonPtr next, prev;
			vector<ofVec3f> nextVec, prevVec;

			if(toEstimate == kPolygonNameTop || toEstimate == kPolygonNameBottom)
			{
				int j = 0;
				do
				{
					next = partialEstimation.at(j);
					prev = getOppositePolygon(next->getPolygonModelObject()->getName(),partialEstimation);//Estimate TOP
					j++;
				}
				while(prev.get() == NULL && j < partialEstimation.size());

				if(prev.get() == NULL)
				{
					missing.remove(toEstimate);
					continue; // no se puede estimar el poligono
				}

				nextVec = next->getPolygonModelObject()->getMathModel().getVertexs();
				prevVec = prev->getPolygonModelObject()->getMathModel().getVertexs();

				if(toEstimate == kPolygonNameBottom)
				{
					sort(nextVec.begin(),nextVec.end(), sortOnYDesc<ofVec3f>);
					sort(prevVec.begin(),prevVec.end(), sortOnYDesc<ofVec3f>);
				}
				else
				{
					sort(nextVec.begin(),nextVec.end(), sortOnYAsc<ofVec3f>);
					sort(prevVec.begin(),prevVec.end(), sortOnYAsc<ofVec3f>);
				}
			}	
			else
			{
				next = getNextPolygon(toEstimate,partialEstimation);
				prev = getPrevPolygon(toEstimate,partialEstimation);

				if(next.get() == NULL)
					next = getPCPolygon(kPolygonNameTop,partialEstimation);
				if(prev.get() == NULL)
					prev = getPCPolygon(kPolygonNameBottom,partialEstimation);

				if(next.get() == NULL || prev.get() == NULL)
				{
					missing.remove(toEstimate);
					continue; // no se puede estimar el poligono
				}

				nextVec = next->getPolygonModelObject()->getMathModel().getVertexs();
				prevVec = prev->getPolygonModelObject()->getMathModel().getVertexs();

				bool ascending = next->getCenter().z < prev->getCenter().z;
				if(ascending)
				{
					sort(nextVec.begin(),nextVec.end(), sortOnXAsc<ofVec3f>);
					sort(prevVec.begin(),prevVec.end(), sortOnXAsc<ofVec3f>);
				}
				else
				{
					sort(nextVec.begin(),nextVec.end(), sortOnXDesc<ofVec3f>);
					sort(prevVec.begin(),prevVec.end(), sortOnXDesc<ofVec3f>);
				}
			}

			vector<ofVec3f> faceVertex;

			faceVertex.insert(faceVertex.end(),nextVec.begin(), nextVec.begin() + 2);
			faceVertex.insert(faceVertex.end(),prevVec.begin(), prevVec.begin() + 2);

			Plane3D plane(faceVertex.at(0), faceVertex.at(1), faceVertex.at(2));
			pcl::ModelCoefficients coef = plane.getCoefficients();
				
			//fix normal
			ofVec3f internalPoint = nextVec.at(3);
			ofVec3f normal = ofVec3f(coef.values.at(0),coef.values.at(1),coef.values.at(2));
			float dist = normal.dot(internalPoint - faceVertex.at(0));
			if(dist > 0)
			{
				normal *= -1;
				Plane3D orientedPlane(faceVertex.at(0),normal);
				coef = orientedPlane.getCoefficients();
			}

			PCPtr faceCloud (new PC());
			for(int i = 0; i < faceVertex.size(); i ++)
			{
				faceCloud->push_back(OFVEC3F_PCXYZ(faceVertex.at(i)));
			}

			//top face correction
			if(fullEstimation && toEstimate == kPolygonNameTop)
			{
				gModel->tableMutex.lock();
				TablePtr t = gModel->getTable();
				gModel->tableMutex.unlock();

				float calcHeight = t->getPolygonModelObject()->getMathModel().distance(PCXYZ_OFVEC3F(faceCloud->at(0)));
				ofVec3f translationCorrection = normal * (height - calcHeight);
					
				PCPtr correctionCloud (new PC());
				Eigen::Transform<float,3,Eigen::Affine> transformationCorrection;
				transformationCorrection = Eigen::Translation<float,3>(translationCorrection.x, translationCorrection.y, translationCorrection.z);
				pcl::transformPointCloud(*faceCloud,*correctionCloud,transformationCorrection);
					
				*faceCloud = *correctionCloud;
				Plane3D orientedPlane(PCXYZ_OFVEC3F(faceCloud->at(0)),normal);
				coef = orientedPlane.getCoefficients();
			}

			PCPolygonPtr pcp(new PCQuadrilateral(coef, faceCloud, 99, true));
			pcp->detectPolygon();
			pcp->getPolygonModelObject()->setName(toEstimate);

					
			partialEstimation.push_back(pcp);
			estimated.push_back(pcp);

			PCPolygonPtr estimatedPol = duplicatePol(pcp,partialEstimation);
			partialEstimation.push_back(estimatedPol);
			estimated.push_back(estimatedPol);
			missing = getMissing(partialEstimation);
			saveCloud("fullEstimatedFace.pcd",*faceCloud);

			i++;
		}
		return estimated;
	}

	// ------------------------------------------------------------------------------
	void PCBox::detectPrimitives() {
		PCPolyhedron::detectPrimitives();
		pcPolygonsMutex.lock();
		if(pcpolygons.size() == 6)
		{
			fullEstimation = true;
			for(int i = 0; i < pcpolygons.size(); i ++)
			{
				IPolygonName name = pcpolygons.at(i)->getPolygonModelObject()->getName();
				switch(name)
				{
					case kPolygonNameTop:
						top = pcpolygons.at(i);
						break;
					case kPolygonNameSideA:
						sideA = pcpolygons.at(i);
						break;
					case kPolygonNameSideB:
						sideB = pcpolygons.at(i);
						break;
					case kPolygonNameSideC:
						sideC = pcpolygons.at(i);
						break;
					case kPolygonNameSideD:
						sideD = pcpolygons.at(i);
						break;
					case kPolygonNameBottom:
						bottom = pcpolygons.at(i);
						break;
				}
			}
			messureBox();
		}
		pcPolygonsMutex.unlock();
	}


	//// Get Polygons
	// ------------------------------------------------------------------------------
	PCPolygonPtr PCBox::getPCPolygon(IPolygonName name, const vector<PCPolygonPtr>& newPolygons)
	{
		for(int i = 0; i < newPolygons.size(); i++)
			if(newPolygons.at(i)->getPolygonModelObject()->getName() == name)
				return newPolygons.at(i);

		return PCPolygonPtr();
	}

	// ------------------------------------------------------------------------------
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
	
	// ------------------------------------------------------------------------------
	PCPolygonPtr PCBox::getNextPolygon(IPolygonName toEstimate, const vector<PCPolygonPtr>& newPolygons)
	{
		IPolygonName searchedPolygon = getNextPolygonName(toEstimate);

		for(int i = 0; i < newPolygons.size(); i++)
			if(newPolygons.at(i)->getPolygonModelObject()->getName() == searchedPolygon)
				return newPolygons.at(i);
		
		return PCPolygonPtr();
	}

	// ------------------------------------------------------------------------------
	PCPolygonPtr PCBox::getPrevPolygon(IPolygonName toEstimate, const vector<PCPolygonPtr>& newPolygons)
	{
		IPolygonName searchedPolygon = getPrevPolygonName(toEstimate);

		for(int i = 0; i < newPolygons.size(); i++)
			if(newPolygons.at(i)->getPolygonModelObject()->getName() == searchedPolygon)
				return newPolygons.at(i);
		
		return PCPolygonPtr();
	}

	// ------------------------------------------------------------------------------
	PCPolygonPtr PCBox::getOppositePolygon(IPolygonName toEstimate, const vector<PCPolygonPtr>& newPolygons)
	{
		IPolygonName searchedPolygon = getOppositePolygonName(toEstimate);

		for(int i = 0; i < newPolygons.size(); i++)
			if(newPolygons.at(i)->getPolygonModelObject()->getName() == searchedPolygon)
				return newPolygons.at(i);
		
		return PCPolygonPtr();
	}

	// ------------------------------------------------------------------------------
	IPolygonName PCBox::getOppositePolygonName(IPolygonName toEstimate)
	{
		switch(toEstimate)
		{
			case kPolygonNameBottom:
				return kPolygonNameTop;
			case kPolygonNameSideA:
				return kPolygonNameSideC;
			case kPolygonNameSideB:
				return kPolygonNameSideD;
			case kPolygonNameSideC:
				return kPolygonNameSideA;
			case kPolygonNameSideD:
				return kPolygonNameSideB;
			case kPolygonNameTop:
				return kPolygonNameBottom;
			case kPolygonNameUnknown:
				return kPolygonNameUnknown;
		}
	}

	// ------------------------------------------------------------------------------
	IPolygonName PCBox::getPrevPolygonName(IPolygonName toEstimate)
	{
		switch(toEstimate)
		{
			case kPolygonNameBottom:
				return kPolygonNameSideB;
			case kPolygonNameSideA:
				return kPolygonNameSideD;
			case kPolygonNameSideB:
				return kPolygonNameSideA;
			case kPolygonNameSideC:
				return kPolygonNameSideB;
			case kPolygonNameSideD:
				return kPolygonNameSideC;
			case kPolygonNameTop:
				return kPolygonNameSideB;
			case kPolygonNameUnknown:
				return kPolygonNameUnknown;
		}
	}

	// ------------------------------------------------------------------------------
	IPolygonName PCBox::getNextPolygonName(IPolygonName toEstimate)
	{
		switch(toEstimate)
		{
			case kPolygonNameBottom:
				return kPolygonNameSideA;
			case kPolygonNameSideA:
				return kPolygonNameSideB;
			case kPolygonNameSideB:
				return kPolygonNameSideC;
			case kPolygonNameSideC:
				return kPolygonNameSideD;
			case kPolygonNameSideD:
				return kPolygonNameSideA;
			case kPolygonNameTop:
				return kPolygonNameSideA;
			case kPolygonNameUnknown:
				return kPolygonNameUnknown;
		}
	}

	//// Estimation
	// ------------------------------------------------------------------------------
	vector<PCPolygonPtr> PCBox::discardPolygonsOutOfBox(const vector<PCPolygonPtr>& toDiscard)
	{
		vector<PCPolygonPtr> polygonsInBox;
		TablePtr table = gModel->getTable();

		//pcl::io::savePCDFile("table.pcd",table->getCloud());
		
		for(int i = 0; i < toDiscard.size(); i++)
		{
			//pcl::io::savePCDFile("pol" + ofToString(i) + ".pcd",toDiscard.at(i)->getCloud());

			PCPtr cloudPtr(toDiscard.at(i)->getCloud());
			if(table->isOnTable(cloudPtr))
			{
				//cout << "pol" << ofToString(i) << " On table!" <<endl;
				polygonsInBox.push_back(toDiscard.at(i));
			}
			else if(table->isParallelToTable(toDiscard.at(i)))
			{
				//cout << "pol" << ofToString(i) << " parallel table!" <<endl;
				polygonsInBox.push_back(toDiscard.at(i));
				//pcl::io::savePCDFile("pol" + ofToString(i) + ".pcd",toDiscard.at(i)->getCloud());
			}

		}

		return polygonsInBox;
	}
	
	// ------------------------------------------------------------------------------
	vector<PCPolygonPtr> PCBox::discardPolygonsOutOfBox(const vector<PCPolygonPtr>& toDiscard, const vector<PCPolygonPtr>& inPolygon)
	{
		vector<PCPolygonPtr> polygonsInBox;
		for(int i = 0; i < toDiscard.size(); i++)
		{
			bool belongsToBox = true;
			Plane3D dPlane = toDiscard.at(i)->getMathPolygonModelApproximation()->getMathModel().getPlane();
			//pcl::io::savePCDFile("pol" + ofToString(i) + ".pcd",toDiscard.at(i)->getCloud());
			for(int j = 0; j < inPolygon.size(); j++)
			{
				if(!dPlane.isPerpendicular(inPolygon.at(j)->getMathPolygonModelApproximation()->getMathModel().getPlane()))
				{
					belongsToBox = false;
					break;
				}
			}
			if(belongsToBox)
				polygonsInBox.push_back(toDiscard.at(i));
			else
				saveCloud("Discared.pcd",*toDiscard.at(i)->getCloud());
		}

		return polygonsInBox;
	}

	// ------------------------------------------------------------------------------
	PCPolygonPtr PCBox::duplicatePol(const PCPolygonPtr& polygon,const vector<PCPolygonPtr>& newPolygons)
	{
		pcl::ModelCoefficients coeff = polygon->getCoefficients();
		PCPtr cloudToMove = polygon->getCloud();
		PCPtr cloudMoved (new PC()); 
		ofVec3f normal;

		//Nombro el poligono a duplicar
		IPolygonName polFatherName = polygon->getPolygonModelObject()->getName();
		IPolygonName polName;

		if(polFatherName == kPolygonNameTop)
			polName = kPolygonNameBottom;
		else if(polFatherName == kPolygonNameBottom)
			polName = kPolygonNameTop;
		else if(polFatherName == kPolygonNameSideA)
			polName = kPolygonNameSideC;
		else if(polFatherName == kPolygonNameSideB)
			polName = kPolygonNameSideD;
		else if(polFatherName == kPolygonNameSideC)
			polName = kPolygonNameSideA;
		else if(polFatherName == kPolygonNameSideD)
			polName = kPolygonNameSideB;

		if(fullEstimation)
		{
			normal = polygon->getNormal();
			ofVec3f traslation;
			
			if(polFatherName == kPolygonNameTop)
				traslation = normal * (-height); 
			else if(polFatherName == kPolygonNameSideA || polFatherName == kPolygonNameSideC)
				traslation = normal * (-depth); 
			else if(polFatherName == kPolygonNameSideB || polFatherName == kPolygonNameSideD)
				traslation = normal * (-width); 

			Eigen::Transform<float,3,Eigen::Affine> transformation;
			transformation = Eigen::Translation<float,3>(traslation.x, traslation.y, traslation.z);

			pcl::transformPointCloud(*cloudToMove,*cloudMoved,transformation);
		}
		else
		{
			PCPolygonPtr f1;

			//Encuentro el poligono 'vecino' al que quiero duplicar
			TablePtr t = gModel->getTable();
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

			if(polName == kPolygonNameBottom)
			{
				coeff =  t->getCoefficients();
				normal = t->getNormal();
				cloudMoved = projectPointsInPlane(cloudToMove, coeff);
			}
			else
			{
				//Busco el largo que tengo que desplazar
				vector<ofVec3f> vex = f1->getPolygonModelObject()->getMathModel().getVertexs();
		
				if(!parallelAndNotInContact && foundFace) // Busco los 2 puntos con menor 'y' para hallar el ancho de la cara
				{
					sort(vex.begin(), vex.end(), sortOnYDesc<ofVec3f>);
				}
				else // Busco los 2 puntos con menor 'x' para hallar el alto de la cara
				{
					sort(vex.begin(),vex.end(), sortOnXDesc<ofVec3f>);
				}
				ofVec3f min1 = vex.at(0);
				ofVec3f min2 = vex.at(1);

				saveCloud("min1_" + ofToString(f1->getPolygonModelObject()->getName()) + ".pcd",min1);
				saveCloud("min2_"+ofToString(f1->getPolygonModelObject()->getName())+".pcd",min2);

				float f1Width = abs((min1 - min2).length());
				normal = polygon->getNormal();
				ofVec3f traslation = normal * (-f1Width); 
				Eigen::Transform<float,3,Eigen::Affine> transformation;
				transformation = Eigen::Translation<float,3>(traslation.x, traslation.y, traslation.z);

				pcl::transformPointCloud(*cloudToMove,*cloudMoved,transformation);
				saveCloud("with" + ofToString(f1->getPolygonModelObject()->getName()) + ".pcd", *f1->getCloud());
			}
		}
		//Corrección de coeficiente
		//invierto la normal
		normal *= -1;
		Plane3D plane(PCXYZ_OFVEC3F(cloudMoved->at(0)), normal);
		coeff = plane.getCoefficients();

		PCPolygonPtr estimatedPol (new PCQuadrilateral(coeff,cloudMoved,polygon->getId()*(-2),true));
		estimatedPol->detectPolygon();
		estimatedPol->getPolygonModelObject()->setName(polName);

		saveCloud("estimated" + ofToString(estimatedPol->getPolygonModelObject()->getName()) + ".pcd", *estimatedPol->getCloud());
		saveCloud("from" + ofToString(polygon->getPolygonModelObject()->getName()) + ".pcd", *polygon->getCloud());
		

		return estimatedPol;
	}

	// ------------------------------------------------------------------------------
	void PCBox::messureBox()
	{
		cout << "Messures of: " << sideA->getPolygonModelObject()->getName() << endl; 
		vector<ofVec3f> sideAVex = sideA->getPolygonModelObject()->getMathModel().getVertexs();
		vector<ofVec3f> sideBVex = sideB->getPolygonModelObject()->getMathModel().getVertexs();
		width = abs((sideAVex.at(1) - sideAVex.at(2)).length());
		cout << "\tw: " << width << endl;
		height = abs((sideAVex.at(0) - sideAVex.at(1)).length());
		cout << "\th: " << height << endl;
		depth = abs((sideBVex.at(1) - sideBVex.at(2)).length());
		cout << "\td: " << depth << endl;

	}

	
}
