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
#include "transformationUtils.h"
#include "utils.h"
#include "Feature.h"


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
	/*	PCPolyhedron::unifyVertexs();
		return;*/

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
					
					
					ofVec3f vertex = plane0.intersection(plane1,plane2);

					/*cout << "dist-0:" << plane0.distance(vertex) << endl;
					cout << "dist-1:" << plane1.distance(vertex) << endl;
					cout << "dist-2:" << plane2.distance(vertex) << endl;*/

					tempVertexs[vertexNames[i][0]][vertexIdx[i][0]] = vertex;
					tempVertexs[vertexNames[i][1]][vertexIdx[i][1]] = vertex;
					tempVertexs[vertexNames[i][2]][vertexIdx[i][2]] = vertex;

					vertexs.push_back(vertex);
					//DEBUG
					/*saveCloud("vmerged" + ofToString(i) + ".pcd", vertex);
					saveCloud("merged" + ofToString(i) + "1.pcd", *p0->getCloud());
					saveCloud("mergedProj" + ofToString(i) + "1.pcd", *projectPointsInPlane(p0->getCloud(),p0->getPolygonModelObject()->getMathModel().getPlane().getCoefficients()));
					saveCloud("merged" + ofToString(i) + "2.pcd", *p1->getCloud());
					saveCloud("mergedProj" + ofToString(i) + "2.pcd", *projectPointsInPlane(p1->getCloud(),p1->getPolygonModelObject()->getMathModel().getPlane().getCoefficients()));
					saveCloud("merged" + ofToString(i) + "3.pcd", *p2->getCloud());
					saveCloud("mergedProj" + ofToString(i) + "3.pcd", *projectPointsInPlane(p2->getCloud(),p2->getPolygonModelObject()->getMathModel().getPlane().getCoefficients()));
					*/

				}
			}
		
			for(int i = 0; i < 6; i++)
			{
				IPolygonName name = (IPolygonName)i;
				PCPolygonPtr p0 = getPCPolygon(name, pcpolygons);
				if(p0.get() != NULL)
					p0->getPolygonModelObject()->setVertexs(tempVertexs[i]);
			}
			saveCloud("UnifiedVertex" + ofToString(getId()) + ".pcd", vertexs);

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
	vector<PCPolygonPtr> PCBox::estimateHiddenPolygons(const vector<PCPolygonPtr>& newPolygons, bool& estimationOk)
	{
		map<IPolygonName, PCPolygonPtr> estimated;
		if(newPolygons.size() <= 0)
		{
			estimationOk = false;
			return vector<PCPolygonPtr>();
		}

		for(int i = 0; i < newPolygons.size(); i ++)
		{
			PCPolygonPtr estimatedPol = duplicatePol(newPolygons.at(i),newPolygons);
			if(estimatedPol == PCPolygonPtr()) // Fallo estimacion
			{
				estimationOk = false;
				return vector<PCPolygonPtr>(); // no se puede estimar el poligono
			}
			estimated[estimatedPol->getPolygonModelObject()->getName()] = estimatedPol;
			saveCloud("estimated" + ofToString(estimatedPol->getPolygonModelObject()->getName()) + ".pcd", *estimatedPol->getCloud());
		}
		
		map<IPolygonName, PCPolygonPtr> partialEstimation = estimated;
		for(int i = 0; i < newPolygons.size(); i++)
			partialEstimation[newPolygons.at(i)->getPolygonModelObject()->getName()] = newPolygons.at(i);
	
		int maxIter = 0;
		int i = 0;
			
		list<IPolygonName> missing = getMissing(partialEstimation);

		while(partialEstimation.size() < 6 && 
				maxIter < 5 &&
				partialEstimation.size() > 1 &&
				missing.size() > 0)
		{
			maxIter++;
			IPolygonName toEstimate = *missing.begin();
			PCPolygonPtr next, prev;
			vector<ofVec3f> nextVec, nextVecToEstimate, prevVec, prevVecToEstimate;

			//cout << "fully estimating: " << toEstimate << endl;

			if(toEstimate == kPolygonNameTop || toEstimate == kPolygonNameBottom)
			{
				map<IPolygonName, PCPolygonPtr>::iterator it = partialEstimation.begin();
				do
				{
					next = (it)->second;
					if(next->getPolygonModelObject()->getName() != kPolygonNameTop &&
					   next->getPolygonModelObject()->getName() != kPolygonNameBottom)
						prev = getOppositePolygon(next->getPolygonModelObject()->getName(),partialEstimation);//Estimate TOP
					it++;
				}
				while(prev.get() == NULL && it != partialEstimation.end());

				if(prev.get() == NULL)
				{
					missing.remove(toEstimate);
					cerr<<"no se puede estimar el poligono " << toEstimate << endl;
					estimationOk = false;
					return vector<PCPolygonPtr>(); // no se puede estimar el poligono
				}

				nextVec = next->getPolygonModelObject()->getMathModel().getVertexs();
				prevVec = prev->getPolygonModelObject()->getMathModel().getVertexs();

				

				if(toEstimate == kPolygonNameBottom)
				{
					if(fullEstimation)
					{
						nextVecToEstimate.push_back(nextVec.at(1));
						nextVecToEstimate.push_back(nextVec.at(2));

						prevVecToEstimate.push_back(prevVec.at(1));
						prevVecToEstimate.push_back(prevVec.at(2));
					}
					else
					{
						if(IsFeatureMoveArmActive())
						{
							nextVec = transformVector(nextVec, gTransformation->getInverseWorldTransformation());
							prevVec = transformVector(prevVec, gTransformation->getInverseWorldTransformation());
						}

						sort(nextVec.begin(), nextVec.end(), sortOnYDesc<ofVec3f>);
						sort(prevVec.begin(), prevVec.end(), sortOnYDesc<ofVec3f>);
						
						if(IsFeatureMoveArmActive())
						{
							nextVec = transformVector(nextVec, gTransformation->getWorldTransformation());
							prevVec = transformVector(prevVec, gTransformation->getWorldTransformation());
						}

						nextVecToEstimate.push_back(nextVec.at(0));
						nextVecToEstimate.push_back(nextVec.at(1));

						prevVecToEstimate.push_back(prevVec.at(0));
						prevVecToEstimate.push_back(prevVec.at(1));
					}
				}
				else
				{
					if(fullEstimation)
					{
						nextVecToEstimate.push_back(nextVec.at(0));
						nextVecToEstimate.push_back(nextVec.at(3));

						prevVecToEstimate.push_back(prevVec.at(0));
						prevVecToEstimate.push_back(prevVec.at(3));
					}
					else
					{
						if(IsFeatureMoveArmActive())
						{
							nextVec = transformVector(nextVec, gTransformation->getInverseWorldTransformation());
							prevVec = transformVector(prevVec, gTransformation->getInverseWorldTransformation());
						}
						sort(nextVec.begin(), nextVec.end(), sortOnYAsc<ofVec3f>);
						sort(prevVec.begin(), prevVec.end(), sortOnYAsc<ofVec3f>);
						if(IsFeatureMoveArmActive())
						{
							nextVec = transformVector(nextVec, gTransformation->getWorldTransformation());
							prevVec = transformVector(prevVec, gTransformation->getWorldTransformation());
						}

						nextVecToEstimate.push_back(nextVec.at(0));
						nextVecToEstimate.push_back(nextVec.at(1));

						prevVecToEstimate.push_back(prevVec.at(0));
						prevVecToEstimate.push_back(prevVec.at(1));
					}
				}
			}	
			else
			{
				next = getNextPolygon(toEstimate,partialEstimation);
				prev = getPrevPolygon(toEstimate,partialEstimation);

				if(next.get() == NULL || prev.get() == NULL)
				{
					missing.remove(toEstimate);
					cerr<<"no se puede estimar el poligono " << toEstimate << endl;
					estimationOk = false;
					return vector<PCPolygonPtr>(); // no se puede estimar el poligono
				}

				nextVec = next->getPolygonModelObject()->getMathModel().getVertexs();
				prevVec = prev->getPolygonModelObject()->getMathModel().getVertexs();

				if(fullEstimation)
				{
					nextVecToEstimate.push_back(nextVec.at(3));
					nextVecToEstimate.push_back(nextVec.at(2));

					prevVecToEstimate.push_back(prevVec.at(0));
					prevVecToEstimate.push_back(prevVec.at(1));
				}
				else
				{
					if(IsFeatureMoveArmActive())
					{
						//cout << "acctivo" << endl;
						nextVec = transformVector(nextVec, gTransformation->getInverseWorldTransformation());
						prevVec = transformVector(prevVec, gTransformation->getInverseWorldTransformation());
					}
					if(toEstimate == kPolygonNameSideA)
					{
						sort(nextVec.begin(), nextVec.end(), sortOnZDesc<ofVec3f>);
						sort(prevVec.begin(), prevVec.end(), sortOnZDesc<ofVec3f>);
					}
					if(toEstimate == kPolygonNameSideC)
					{
						sort(nextVec.begin(), nextVec.end(), sortOnZAsc<ofVec3f>);
						sort(prevVec.begin(), prevVec.end(), sortOnZAsc<ofVec3f>);
					}
					if(toEstimate == kPolygonNameSideB)
					{
						sort(nextVec.begin(), nextVec.end(), sortOnXDesc<ofVec3f>);
						sort(prevVec.begin(), prevVec.end(), sortOnXDesc<ofVec3f>);
					}
					if(toEstimate == kPolygonNameSideD)
					{
						sort(nextVec.begin(), nextVec.end(), sortOnXAsc<ofVec3f>);
						sort(prevVec.begin(), prevVec.end(), sortOnXAsc<ofVec3f>);
					}

					if(IsFeatureMoveArmActive())
					{
						nextVec = transformVector(nextVec, gTransformation->getWorldTransformation());
						prevVec = transformVector(prevVec, gTransformation->getWorldTransformation());
					}
					nextVecToEstimate.push_back(nextVec.at(0));
					nextVecToEstimate.push_back(nextVec.at(1));

					prevVecToEstimate.push_back(prevVec.at(0));
					prevVecToEstimate.push_back(prevVec.at(1));
				}
			}

			vector<ofVec3f> faceVertex;

			faceVertex.insert(faceVertex.end(),nextVecToEstimate.begin(), nextVecToEstimate.begin() + 2);
			faceVertex.insert(faceVertex.end(),prevVecToEstimate.begin(), prevVecToEstimate.begin() + 2);

			Plane3D plane(faceVertex.at(0), faceVertex.at(1), faceVertex.at(2));
			pcl::ModelCoefficients coef = plane.getCoefficients();
				
			//fix normal
			ofVec3f internalPoint = (nextVec.at(3) + nextVec.at(1))/2; // Punto medio de la cara
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

			////top face correction
			//if(fullEstimation && toEstimate == kPolygonNameTop)
			//{
			//	gModel->tableMutex.lock();
			//	TablePtr t = gModel->getTable();
			//	gModel->tableMutex.unlock();

			//	float calcHeight = t->getPolygonModelObject()->getMathModel().distance(PCXYZ_OFVEC3F(faceCloud->at(0)));
			//	ofVec3f translationCorrection = normal * (height - calcHeight);
			//		
			//	PCPtr correctionCloud (new PC());
			//	Eigen::Transform<float,3,Eigen::Affine> transformationCorrection;
			//	transformationCorrection = Eigen::Translation<float,3>(translationCorrection.x, translationCorrection.y, translationCorrection.z);
			//	pcl::transformPointCloud(*faceCloud,*correctionCloud,transformationCorrection);
			//		
			//	*faceCloud = *correctionCloud;
			//	Plane3D orientedPlane(PCXYZ_OFVEC3F(faceCloud->at(0)),normal);
			//	coef = orientedPlane.getCoefficients();
			//}

			PCPolygonPtr pcp(new PCQuadrilateral(coef, faceCloud, 99, true));
			pcp->detectPolygon();
			pcp->getPolygonModelObject()->setName(toEstimate);
			fixVertexsOrder(pcp,toEstimate);
					
			partialEstimation[toEstimate] = pcp;
			estimated[toEstimate] = pcp;

			if(!isFaceOccluded(toEstimate))
			{
				PCPolygonPtr estimatedPol = duplicatePol(pcp,partialEstimation);
				partialEstimation[estimatedPol->getPolygonModelObject()->getName()] = estimatedPol;
				estimated[estimatedPol->getPolygonModelObject()->getName()] = estimatedPol;
			}
			missing = getMissing(partialEstimation);
			saveCloud("fullEstimatedFace.pcd",*faceCloud);

			i++;
		}

		vector<PCPolygonPtr> estimatedVector;
		for(map<IPolygonName,PCPolygonPtr>::const_iterator it = estimated.begin(); it != estimated.end(); ++it)
			estimatedVector.push_back(it->second);
		estimationOk = true;
		return estimatedVector;
	}

	// ------------------------------------------------------------------------------
	void PCBox::detectPrimitives() {
		fullEstimation = false;
		pcpolygons.clear();
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

			//Corrijo normales?

			measures = measureBox();
			

			//Valido volumen del objeto
			const float OBJECT_MIN_VOLUME = 0.000125;
			isvalid = measures.x * measures.y * measures.z > OBJECT_MIN_VOLUME;
			fullEstimation = isvalid;
			if(fullEstimation)
				cout << "+++++++++++++++++ full estimation +++++++++++++++" << endl;


		}
		else
		{
			isvalid = false;
		}
		pcPolygonsMutex.unlock();
	}


	//// Get Polygons
	// ------------------------------------------------------------------------------
	PCPolygonPtr PCBox::getPCPolygon(IPolygonName name, const map<IPolygonName,PCPolygonPtr>& estimated)
	{
		map<IPolygonName,PCPolygonPtr>::const_iterator it = estimated.find(name);
		if(it == estimated.end())
			return PCPolygonPtr();
		else
			return it->second;
	}

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
	list<IPolygonName> PCBox::getMissing(const map<IPolygonName,PCPolygonPtr>& estimated)
	{
		list<IPolygonName> missing;
		if(estimated.find(kPolygonNameTop) == estimated.end())
			missing.push_back(kPolygonNameTop);
		if(estimated.find(kPolygonNameSideA) == estimated.end())
			missing.push_back(kPolygonNameSideA);
		if(estimated.find(kPolygonNameSideB) == estimated.end())
			missing.push_back(kPolygonNameSideB);
		if(estimated.find(kPolygonNameSideC) == estimated.end())
			missing.push_back(kPolygonNameSideC);
		if(estimated.find(kPolygonNameSideD) == estimated.end())
			missing.push_back(kPolygonNameSideD);
		if(estimated.find(kPolygonNameBottom) == estimated.end())
			missing.push_back(kPolygonNameBottom);

		return missing;
	}
	
	// ------------------------------------------------------------------------------
	PCPolygonPtr PCBox::getNextPolygon(IPolygonName toEstimate, const map<IPolygonName,PCPolygonPtr>& estimated)
	{
		IPolygonName searchedPolygon = getNextPolygonName(toEstimate);

		map<IPolygonName,PCPolygonPtr>::const_iterator it = estimated.find(searchedPolygon);
		if(it == estimated.end())
			return PCPolygonPtr();
		else
			return it->second;
	}

	// ------------------------------------------------------------------------------
	PCPolygonPtr PCBox::getPrevPolygon(IPolygonName toEstimate, const map<IPolygonName,PCPolygonPtr>& estimated)
	{
		IPolygonName searchedPolygon = getPrevPolygonName(toEstimate);

		map<IPolygonName,PCPolygonPtr>::const_iterator it = estimated.find(searchedPolygon);
		if(it == estimated.end())
			return PCPolygonPtr();
		else
			return it->second;
	}

	// ------------------------------------------------------------------------------
	PCPolygonPtr PCBox::getOppositePolygon(IPolygonName toEstimate, const map<IPolygonName,PCPolygonPtr>& estimated)
	{
		IPolygonName searchedPolygon = getOppositePolygonName(toEstimate);
		map<IPolygonName,PCPolygonPtr>::const_iterator it = estimated.find(searchedPolygon);
		if(it == estimated.end())
			return PCPolygonPtr();
		else
			return it->second;
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
		return toDiscard;
		//vector<PCPolygonPtr> polygonsInBox;
		//TablePtr table = gModel->getTable();

		////pcl::io::savePCDFile("table.pcd",table->getCloud());
		//
		//for(int i = 0; i < toDiscard.size(); i++)
		//{
		//	//pcl::io::savePCDFile("pol" + ofToString(i) + ".pcd",toDiscard.at(i)->getCloud());

		//	PCPtr cloudPtr(toDiscard.at(i)->getCloud());
		//	if(table->isOnTable(cloudPtr))
		//	{
		//		//cout << "pol" << ofToString(i) << " On table!" <<endl;
		//		polygonsInBox.push_back(toDiscard.at(i));
		//	}
		//	else if(table->isParallelToTable(toDiscard.at(i)))
		//	{
		//		//cout << "pol" << ofToString(i) << " parallel table!" <<endl;
		//		polygonsInBox.push_back(toDiscard.at(i));
		//		//pcl::io::savePCDFile("pol" + ofToString(i) + ".pcd",toDiscard.at(i)->getCloud());
		//	}

		//}

		//return polygonsInBox;
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
	PCPolygonPtr PCBox::duplicatePol(const PCPolygonPtr& polygon,const map<IPolygonName,PCPolygonPtr>& estimated)
	{

		pcl::ModelCoefficients coeff = polygon->getCoefficients();
		PCPtr cloudToMove = polygon->getCloud();
		PCPtr cloudMoved (new PC()); 
		ofVec3f normal;

		//Nombro el poligono a duplicar
		IPolygonName polFatherName = polygon->getPolygonModelObject()->getName();
		IPolygonName polName;

		//cout << "Duplicating: " << polFatherName << endl;

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
			
			if(polFatherName == kPolygonNameTop || polFatherName == kPolygonNameBottom)
			{
				/*ofVec3f n1 = getPCPolygon(kPolygonNameSideA,estimated)->getNormal();
				ofVec3f n2 = getPCPolygon(kPolygonNameSideB,estimated)->getNormal();
				normal = n1.crossed(n2);
				
				if(polFatherName == kPolygonNameBottom)
					normal *= -1;
*/
				traslation = normal * (-measures.y); 
			}
			else if(polFatherName == kPolygonNameSideA || polFatherName == kPolygonNameSideC)
			{
				/*ofVec3f n1 = getPCPolygon(kPolygonNameTop,estimated)->getNormal();
				ofVec3f n2 = getPCPolygon(kPolygonNameSideB,estimated)->getNormal();
				normal = n1.crossed(n2);
				
				if(polFatherName == kPolygonNameSideA)
					normal *= -1;
*/
				traslation = normal * (-measures.z); 
			}
			else if(polFatherName == kPolygonNameSideB || polFatherName == kPolygonNameSideD)
			{
				/*ofVec3f n1 = getPCPolygon(kPolygonNameTop,estimated)->getNormal();
				ofVec3f n2 = getPCPolygon(kPolygonNameSideA,estimated)->getNormal();
				normal = n1.crossed(n2);
				
				if(polFatherName == kPolygonNameSideD)
					normal *= -1;*/
				traslation = normal * (-measures.x); 
			}
			Eigen::Transform<float,3,Eigen::Affine> transformation;
			transformation = Eigen::Translation<float,3>(traslation.x, traslation.y, traslation.z);

			pcl::transformPointCloud(*cloudToMove,*cloudMoved,transformation);
		}
		else
		{
			//Encuentro el poligono 'vecino' al que quiero duplicar
			PCPolygonPtr f1;
			bool useTop = false;
			if(polFatherName != kPolygonNameTop)
			{
				f1 = getNextPolygon(polFatherName, estimated);
				if(f1 == PCPolygonPtr())
					f1 = getPrevPolygon(polFatherName, estimated);
				if(f1 == PCPolygonPtr())
				{
					f1 = getPCPolygon(kPolygonNameTop, estimated);
					useTop = true;
				}
				if(f1 == PCPolygonPtr()) // no se puede duplicar
					return PCPolygonPtr();

				vector<ofVec3f> vex = f1->getPolygonModelObject()->getMathModel().getVertexs();
				
				

				if(useTop)
				{
					if(IsFeatureMoveArmActive())
					vex = transformVector(vex, gTransformation->getInverseWorldTransformation());
					// Busco los 2 puntos con menor 'x'
					sort(vex.begin(), vex.end(), sortOnXDesc<ofVec3f>);		
					if(IsFeatureMoveArmActive())
					vex = transformVector(vex, gTransformation->getWorldTransformation());
				}
				else
				{
					// Busco los 2 puntos con menor 'y' 
					//sort(vex.begin(), vex.end(), sortOnYDesc<ofVec3f>);

					// Busco los 2 puntos más cercanos a la mesa
					struct VertexDistance
					{
						VertexDistance(ofVec3f vec, float dist) : vec(vec), dist(dist) { }
						ofVec3f	vec;
						float	dist;
					};
					gModel->tableMutex.lock();
					Plane3D tPlane = gModel->getTable()->getMathModelApproximation()->getPolygons().at(0)->getMathModel().getPlane();
					gModel->tableMutex.unlock();

					vector<VertexDistance> closerToTable;
					for(int i = 0; i < vex.size(); i++)
						closerToTable.push_back(VertexDistance(vex.at(i),tPlane.distance(vex.at(i))));

					sort(closerToTable.begin(),closerToTable.end(),sortOnDistAsc<VertexDistance>);
					vex.clear();
					vex.push_back(closerToTable.at(0).vec);
					vex.push_back(closerToTable.at(1).vec);
				}
				
				

				ofVec3f min1 = vex.at(0);
				ofVec3f min2 = vex.at(1);

				//traslado la nube
				float f1Width = abs((min1 - min2).length());
				normal = polygon->getNormal();
				ofVec3f traslation = normal * (-f1Width); 
				Eigen::Transform<float,3,Eigen::Affine> transformation;
				transformation = Eigen::Translation<float,3>(traslation.x, traslation.y, traslation.z);

				pcl::transformPointCloud(*cloudToMove,*cloudMoved,transformation);

			}
			else
			{
				gModel->tableMutex.lock();
				gModel->getTable()->getCoefficients();
				float dist =  gModel->getTable()->getMathModelApproximation()->getPolygons().at(0)->getMathModel().distance(polygon->getCenter());
				gModel->tableMutex.unlock();

				normal = polygon->getNormal();
				ofVec3f traslation = normal * (-dist); 
				Eigen::Transform<float,3,Eigen::Affine> transformation;
				transformation = Eigen::Translation<float,3>(traslation.x, traslation.y, traslation.z);
				pcl::transformPointCloud(*cloudToMove,*cloudMoved,transformation);

				/*gModel->tableMutex.lock();
				gModel->getTable()->getCoefficients();
				coeff =  gModel->getTable()->getCoefficients();
				gModel->tableMutex.unlock();
				normal = polygon->getNormal();
				cloudMoved = projectPointsInPlane(cloudToMove, coeff);*/
			}

			
			/////**********************************/////////////
			
		}

		//Corrección de coeficiente
		//invierto la normal
		normal *= -1;
		Plane3D plane(PCXYZ_OFVEC3F(cloudMoved->at(0)), normal);
		coeff = plane.getCoefficients();

		PCPolygonPtr estimatedPol (new PCQuadrilateral(coeff,cloudMoved,polygon->getId()*(-2),true));
		estimatedPol->detectPolygon();
		estimatedPol->getPolygonModelObject()->setName(polName);

		// FIX para respetar el orden de los vertices
		fixVertexsOrder(estimatedPol,polName);
		
		saveCloud("estimated" + ofToString(estimatedPol->getPolygonModelObject()->getName()) + ".pcd", *estimatedPol->getCloud());
		saveCloud("from" + ofToString(polygon->getPolygonModelObject()->getName()) + ".pcd", *polygon->getCloud());
		saveCloud("fromVertex" + ofToString(polygon->getPolygonModelObject()->getName()) + ".pcd", polygon->getPolygonModelObject()->getMathModel().getVertexs());
		

		return estimatedPol;


	}

	// ------------------------------------------------------------------------------
	PCPolygonPtr PCBox::duplicatePol(const PCPolygonPtr& polygon,const vector<PCPolygonPtr>& newPolygons)
	{
		map<IPolygonName,PCPolygonPtr> estimated;
		for(vector<PCPolygonPtr>::const_iterator it = newPolygons.begin(); it != newPolygons.end(); ++it)
			estimated[(*it)->getPolygonModelObject()->getName()] = (*it);
		return duplicatePol(polygon,estimated);
	}

	// ------------------------------------------------------------------------------
	ofVec3f PCBox::measureBox()
	{
		float w,h,d;
		//cout << "Measures of: " << sideA->getPolygonModelObject()->getName() << endl; 
		vector<ofVec3f> sideAVex = getPCPolygon(kPolygonNameSideA, pcpolygons)->getPolygonModelObject()->getMathModel().getVertexs();
		vector<ofVec3f> sideBVex = getPCPolygon(kPolygonNameSideB, pcpolygons)->getPolygonModelObject()->getMathModel().getVertexs();
		w = abs((sideAVex.at(1) - sideAVex.at(2)).length());
		//cout << "\tw: " << w << endl;
		h = abs((sideAVex.at(0) - sideAVex.at(1)).length());
		//cout << "\th: " << h << endl;
		d = abs((sideBVex.at(1) - sideBVex.at(2)).length());
		//cout << "\td: " << d << endl;

		return ofVec3f(w,h,d);
	}

	// ------------------------------------------------------------------------------
	void PCBox::fixVertexsOrder(PCPolygonPtr& pol, IPolygonName polName)
	{
		// FIX para respetar el orden de los vertices
		PCPolygonPtr prevEstimation = getPCPolygon(polName,pcpolygons);
		if(prevEstimation != PCPolygonPtr())
		{
			vector<ofVec3f> prevVecs = prevEstimation->getPolygonModelObject()->getMathModel().getVertexs();
			vector<ofVec3f> newVecs = pol->getPolygonModelObject()->getMathModel().getVertexs();
			pol->getPolygonModelObject()->setVertexs(prevVecs);
			pol->getPolygonModelObject()->setVertexsOrdered(newVecs);
		}
	}

	bool PCBox::validate()
	{
		ofVec3f newMeasure = measureBox();
		/*cout << "Measures of: " << sideA->getPolygonModelObject()->getName() << endl; 
		cout << "\tw: " << newMeasure.x << endl;
		cout << "\th: " << newMeasure.y << endl;
		cout << "\td: " << newMeasure.z << endl;*/

		const float OBJECT_MIN_VOLUME = 0.000125;
		bool sizeValid = newMeasure.x < 0.4 && newMeasure.y < 0.4 && newMeasure.z < 0.4; 
		bool volValid = newMeasure.x * newMeasure.y * newMeasure.z > OBJECT_MIN_VOLUME;
		bool vertexValid = this->vertexs.size() == 8;
		bool prevMeasureValid = (newMeasure - measures).length() < 0.05;
		return vertexValid && prevMeasureValid && volValid && sizeValid;
	}

	float PCBox::getVolume()
	{
		return measures.x * measures.y * measures.z;
	}

	
}
