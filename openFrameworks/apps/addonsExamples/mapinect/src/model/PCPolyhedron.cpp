#include "PCPolyhedron.h"

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

#include "ofUtils.h"

#include "Constants.h"
#include "Globals.h"
#include "ofVecUtils.h"
#include "PCQuadrilateral.h"
#include "pointUtils.h"
#include "utils.h"


#define MAX_FACES		3

namespace mapinect {

	PCPolyhedron::PCPolyhedron(const PCPtr& cloud, int objId)
				: PCModelObject(cloud, objId)
	{
	}

	bool hasNoMatching(const PCPolygonPtr& p) {
		return !(p->hasMatching());
	}

	PCPtr PCPolyhedron::getCurrentVertex()
	{
		PCPtr cloud (new PC());
		/*///DEBUG
		vector<ofVec3f> vecList;
		for (vector<PCPolygonPtr>::iterator nextIter = pcpolygons.begin(); nextIter != pcpolygons.end(); nextIter++) {
			Polygon* polygon = (*nextIter)->getPolygonModelObject();
			for (int j = 0; j < polygon->getVertexCount(); j++){
				ofVec3f v(polygon->getVertex(j));
				cloud->push_back(OFXVEC3F_POINTXYZ(v));
			}
		}*/
		return cloud;
	}

	void PCPolyhedron::mergePolygons(vector<PCPolygonPtr>& toMerge) {
		vector<PCPolygonPtr> aAgregar;
		vector<PCPolygonPtr> aProcesar;

		int remainingIters = 10;
		do {
			for (int i = 0; i < toMerge.size(); i++) {
				PCPolygonPtr removed;
				bool		wasRemoved = false;
				bool		fitted = findBestFit(toMerge[i], removed, wasRemoved);

				if (wasRemoved) {
					aProcesar.push_back(removed);
				}
				if (!fitted) {
					aAgregar.push_back(toMerge[i]);
				}
			}
			toMerge = aProcesar;
			aProcesar.clear();
			remainingIters--;
		} while (toMerge.size() > 0 && remainingIters > 0);

		vector<PCPolygonPtr> keep;
		vector<PCPolygonPtr> adjust;
		for (vector<PCPolygonPtr>::iterator iter = pcpolygons.begin(); iter != pcpolygons.end(); iter++) {
			if ((*iter)->hasMatching())
				keep.push_back(*iter);
			else
				adjust.push_back(*iter);
		}
		
		pcpolygons = keep;

		updatePolygons();


		////Ajusto los poligonos que no se ven segun las transformaciones de alguno de los poligonos (?)
		//if(keep.size() > 0)
		//{
		//	Eigen::Transform<float,3,Eigen::Affine> transformation = keep.at(0)->getMatchingTransformation();
		//	for(int i = 0; i < adjust.size(); i++)
		//	{
		//		adjust.at(i)->applyTransformation(&transformation);
		//	}
		//	keep.insert(keep.end(),adjust.begin(),adjust.end());
		//}


		for (int i = 0; i < aAgregar.size(); i++) {
			if (indexOf(pcpolygons, aAgregar[i]) < 0) {
				pcpolygons.push_back(aAgregar[i]);
			}
		}

		aAgregar.clear();

		cout << "pols: " << pcpolygons.size() << endl;

		polygonsCache.clear();
		for (vector<PCPolygonPtr>::iterator p = pcpolygons.begin(); p != pcpolygons.end(); ++p)
		{
			polygonsCache.push_back((*p)->getPolygonModelObject());
		}
	}

	bool xAxisSort (PCPolygonPtr i,PCPolygonPtr j) 
	{ 
		return i->getCenter().x < j->getCenter().x;
	}

	vector<PCPolygonPtr> PCPolyhedron::detectPolygons(const PCPtr& cloud, float planeTolerance, float pointsTolerance, bool limitFaces){
		PCPtr cloudTemp(cloud);
		
		float maxFaces = limitFaces ? MAX_FACES : 100;
		sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2());
		sensor_msgs::PointCloud2::Ptr cloud_filtered_blob (new sensor_msgs::PointCloud2);
		PCPtr cloud_p (new PC());
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		std::vector<ofVec3f> vCloudHull;

		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (50);
		seg.setDistanceThreshold (planeTolerance); //original: 0.01

		// Create the filtering object
		int i = 0, nr_points = cloudTemp->points.size ();
		// mientras 7% de la nube no se haya procesado

		vector<PCPolygonPtr> nuevos;

		int numFaces = 0;
		
		while (cloudTemp->points.size () > 0.07 * nr_points && numFaces < maxFaces)
		{
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud (cloudTemp);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0) {
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}

			//FIX
			PCPtr cloud_filtered_temp_inliers (new PC());
			PCPtr cloud_filtered_temp_outliers (new PC());
			if(inliers->indices.size() != cloudTemp->size())
			{
				// Extract the inliers
				extract.setInputCloud (cloudTemp);
				extract.setIndices (inliers);
				extract.setNegative (false);
				extract.filter (*cloud_filtered_temp_inliers);
				cloud_p = cloud_filtered_temp_inliers;
			}
			else
				cloud_p = cloudTemp;
		
			// Create the filtering object
			extract.setInputCloud (cloudTemp);
			extract.setIndices (inliers);
			extract.setNegative (true);

		
			if(cloud_p->size() != cloudTemp->size())
				extract.filter (*cloud_filtered_temp_outliers);

			cloudTemp = cloud_filtered_temp_outliers;

			saveCloudAsFile("prefilter_pol" + ofToString(i) + ".pcd",*cloud_p);
			
			//Remove outliers by clustering
			PCPtr cloud_p_filtered (new PC());
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud (cloud_p);

			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
			ec.setClusterTolerance (0.02); 
			ec.setMinClusterSize (5);
			ec.setMaxClusterSize (10000);
			ec.setSearchMethod (tree);
			ec.setInputCloud(cloud_p);

			ec.extract (cluster_indices);
			int debuccount = 0;

			if(cluster_indices.size() > 0)
			{
				for (std::vector<int>::const_iterator pit = cluster_indices.at(0).indices.begin (); pit != cluster_indices.at(0).indices.end (); pit++)
					cloud_p_filtered->points.push_back (cloud_p->points[*pit]); //*
			}

			saveCloudAsFile("postfilter_pol" + ofToString(i) + ".pcd",*cloud_p_filtered);
			if (cloud_p_filtered->size() < 4)
				break;

			PCPolygonPtr pcp(new PCQuadrilateral(*coefficients, cloud_p_filtered));
			pcp->detectPolygon();
			pcp->getPolygonModelObject()->setContainer(this);
			
			/*if(gModel->table->isParallelToTable(pcp))
				pcp->getPolygonModelObject()->setName(kPolygonNameTop);
			else
			{
				pcp->getPolygonModelObject()->setName((IPolygonName)sideFacesName);
				sideFacesName++;
			}*/

			nuevos.push_back(pcp);
			
			//writer.write<pcl::PointXYZ> ("cloud_pTemp" + ofToString(i) + ".pcd", *cloud_pTemp, false);
			i++;
			numFaces++;
		}
		
		int sideFacesName = 1;
		sort(nuevos.begin(), nuevos.end(), xAxisSort);
		for(int i = 0; i < nuevos.size(); i++)
		{
			if(gModel->table->isParallelToTable(nuevos.at(i)))
				nuevos.at(i)->getPolygonModelObject()->setName(kPolygonNameTop);
			else
			{
				nuevos.at(i)->getPolygonModelObject()->setName((IPolygonName)sideFacesName);
				sideFacesName++;
			}
		}


		return nuevos;
	}

	void PCPolyhedron::detectPrimitives() {
		vector<PCPolygonPtr> nuevos = detectPolygons(cloud); 
		mergePolygons(nuevos);
		unifyVertexs();
	}

	void PCPolyhedron::updatePolygons() {
		for (int i = 0; i < pcpolygons.size(); i++) {
			pcpolygons[i]->updateMatching();
		}
	}

	void PCPolyhedron::unifyVertexs() {
		typedef struct {
			PCPolygon*		pcp;
			int				vertex;
		} VertexInPCPolygon;
		vector<VertexInPCPolygon> updateVertexs;

		for (vector<PCPolygonPtr>::iterator nextIter = pcpolygons.begin(); nextIter != pcpolygons.end();) {
			vector<PCPolygonPtr>::iterator iter = nextIter++;
			Polygon* polygon = (*iter)->getPolygonModelObject();

			if (polygon == NULL) {
				// No model object is available yet, quit!
				return;
			}

			for (int j = 0; j < polygon->getVertexs().size(); j++) {
				updateVertexs.clear();
				VertexInPCPolygon vpp;
				vpp.pcp = iter->get();
				vpp.vertex = j;
				updateVertexs.push_back(vpp);
				ofVec3f v(polygon->getVertexs()[j]);

				for (vector<PCPolygonPtr>::iterator iter2 = iter; iter2 != pcpolygons.end(); iter2++) {
					Polygon* polygon2 = (*iter2)->getPolygonModelObject();
					for (int k = 0; k < polygon2->getVertexs().size(); k++) {
						ofVec3f v2(polygon2->getVertexs()[k]);
						if (!(v == v2)
							&& polygon->getVertexs()[j].distance(polygon2->getVertexs()[k]) <= MAX_UNIFYING_DISTANCE) {
							VertexInPCPolygon vpp2;
							vpp2.pcp = iter2->get();
							vpp2.vertex = k;
							updateVertexs.push_back(vpp2);
						}
					}
				}

				if (updateVertexs.size() > 1) {
					ofVec3f avg(0, 0, 0);
					for (int i = 0; i < updateVertexs.size(); i++) {
						avg += updateVertexs.at(i).pcp->getPolygonModelObject()->getVertexs()[updateVertexs.at(i).vertex];
					}
					avg /= updateVertexs.size();
					for (int i = 0; i < updateVertexs.size(); i++) {
						updateVertexs.at(i).pcp->getPolygonModelObject()->setVertex(updateVertexs.at(i).vertex, avg);
					}
				}
			}
		}

		/*///DEBUG
		vector<ofVec3f> vecList;
		for (vector<PCPolygonPtr>::iterator nextIter = pcpolygons.begin(); nextIter != pcpolygons.end(); nextIter++) {
			Polygon* polygon = (*nextIter)->getPolygonModelObject();
			for (int j = 0; j < polygon->getVertexCount(); j++){
				ofVec3f v(polygon->getVertex(j));
				vecList.push_back(v);
			}
		}
		createCloud(vecList,"UnifiedVertex.pcd");*/
	}

	bool PCPolyhedron::findBestFit(const PCPolygonPtr& polygon, PCPolygonPtr& removed, bool& wasRemoved)
	{
		for (int i = 0; i < pcpolygons.size(); i++) {
			if (pcpolygons[i]->matches(polygon, removed, wasRemoved)) {
				return true;
			}
		}
		return false;
	}

	void PCPolyhedron::draw() {
		for (vector<PCPolygonPtr>::iterator iter = pcpolygons.begin(); iter != pcpolygons.end(); iter++) {
			(*iter)->draw();
		}
	}

	void PCPolyhedron::applyTransformation()
	{
		PCModelObject::applyTransformation();
		for(vector<PCPolygonPtr>::iterator iter = pcpolygons.begin(); iter != pcpolygons.end(); iter++){
			(*iter)->applyTransformation(&transformation);
		}
	}

	const PCPolygonPtr& PCPolyhedron::getPCPolygon(int index)
	{
		return pcpolygons[index];
	}

	int PCPolyhedron::getPCPolygonSize()
	{
		return pcpolygons.size();
	}

	void PCPolyhedron::resetLod() {
		PCModelObject::resetLod();
		for (int i = 0; i < pcpolygons.size(); i++) {
			pcpolygons[i]->resetLod();
		}
	}

	void PCPolyhedron::increaseLod() {
		PCModelObject::increaseLod();

		for(vector<PCPolygonPtr>::iterator iter = pcpolygons.begin(); iter != pcpolygons.end(); iter++){
			(*iter)->increaseLod(cloud);
		}
		unifyVertexs();
	}

	vector<PCPolygonPtr>	PCPolyhedron::discardPolygonsOutOfBox(const vector<PCPolygonPtr>& toDiscard)
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
	PCPolygonPtr PCPolyhedron::duplicatePol(const PCPolygonPtr& polygon,const vector<PCPolygonPtr>& newPolygons)
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
		ofVec3f min1(numeric_limits<float>::max(),numeric_limits<float>::max(),numeric_limits<float>::max());
		ofVec3f min2(numeric_limits<float>::max(),numeric_limits<float>::max(),numeric_limits<float>::max());
		
		if(!parallelAndNotInContact && foundFace) // Busco los 2 puntos con menor 'y' para hallar el ancho de la cara
		{
			for(int i = 0; i < vex.size(); i ++)
			{
				if(vex.at(i).y < min1.y)
					min1 = vex.at(i);
				else if(vex.at(i).y < min2.y && vex.at(i) != min1)
					min2 = vex.at(i);
			}
		}
		else // Busco los 2 puntos con menor 'x' para hallar el alto de la cara
		{
			for(int i = 0; i < vex.size(); i ++)
			{
				if(vex.at(i).x < min1.x)
					min1 = vex.at(i);
				else if(vex.at(i).x < min2.x && vex.at(i) != min1)
					min2 = vex.at(i);
			}
		}

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

	vector<PCPolygonPtr> PCPolyhedron::estimateHiddenPolygons(const vector<PCPolygonPtr>& newPolygons)
	{
		vector<PCPolygonPtr> estimated;
		if(newPolygons.size() <= 1)
			return estimated;

		for(int i = 0; i < newPolygons.size(); i ++)
		{
			PCPolygonPtr estimatedPol = duplicatePol(newPolygons.at(i),newPolygons);
			estimated.push_back(estimatedPol);
			//saveCloudAsFile ("estimated" + ofToString(i) + ".pcd", *estimatedPol->getCloud());
		}
		
		return estimated;
	}

	void PCPolyhedron::addToModel(const PCPtr& nuCloud)
	{
		PCModelObject::addToModel(nuCloud);

		///TODO:
		/// 1 - Detectar caras de la nueva nube
		/// 2 - Descartar caras que no pertenecen a la caja (caras de la mano)
		/// 3 - Matchear caras de la nube anterior con las nuevas caras
		/// 4 - Estimar caras ocultas (?)
		
		//Detecto nuevas caras
		vector<PCPolygonPtr> nuevos = detectPolygons(nuCloud,0.003,2.6,false); 

		cout << "pre discard: " << nuevos.size() << endl;
		//Descarto caras fuera del objeto
		vector<PCPolygonPtr> inBox = discardPolygonsOutOfBox(nuevos);
		
		cout << "post discard: " << inBox.size() << endl;

		vector<PCPolygonPtr> estimated = estimateHiddenPolygons(inBox);
		inBox.insert(inBox.end(),estimated.begin(),estimated.end());
		mergePolygons(inBox);
		unifyVertexs();
		
		/*for(int i = 0; i < pcpolygons.size(); i ++)
		{
			saveCloudAsFile ("pol" + ofToString(pcpolygons.at(i)->getPolygonModelObject()->getName()) + ".pcd", *pcpolygons.at(i)->getCloud());
		}*/
	}

	void PCPolyhedron::setAndUpdateCloud(const PCPtr& cloud)
	{
		setCloud(cloud);

		//*nuCloud += *cloudPtr;
		//for(vector<PCPolygonPtr>::iterator iter = pcpolygons.begin(); iter != pcpolygons.end(); iter++){
		//	//(*iter)->applyTransformation(&transformation);
		//	(*iter)->increaseLod(nuCloud);
		//}
		//unifyVertexs();
		//pcl::io::savePCDFileASCII ("merged.pcd", *nuCloud);

	}

	const IPolygon* PCPolyhedron::getPolygon(const IPolygonName& name)
	{
		for (vector<IPolygon*>::iterator iter = polygonsCache.begin(); iter != polygonsCache.end(); ++iter)
		{
			if ((*iter)->getName() == name)
			{
				return *iter;
			}
		}
		return NULL;
	}

}
