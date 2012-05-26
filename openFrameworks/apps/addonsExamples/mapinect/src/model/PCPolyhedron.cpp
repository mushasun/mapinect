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
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

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
		partialEstimation = false;
	}

	vector<PCPolygonPtr> PCPolyhedron::mergePolygons(vector<PCPolygonPtr>& toMerge) {
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
		
		//pcpolygons = keep;

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
				keep.push_back(aAgregar[i]);
			}
		}

		aAgregar.clear();

		cout << "pols: " << pcpolygons.size() << endl;

		polygonsCache.clear();
		for (vector<PCPolygonPtr>::iterator p = keep.begin(); p != keep.end(); ++p)
		{
			polygonsCache.push_back((*p)->getPolygonModelObject());
		}

		return keep;
	}
	
	//vector<PCPolygonPtr> PCPolyhedron::detectPolygons(const PCPtr& cloud, float planeTolerance, float pointsTolerance, bool limitFaces){
	//	PCPtr cloudTemp(cloud);
	//	
	//	float maxFaces = limitFaces ? MAX_FACES : 100;
	//	sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2());
	//	sensor_msgs::PointCloud2::Ptr cloud_filtered_blob (new sensor_msgs::PointCloud2);
	//	PCPtr cloud_p (new PC());
	//	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	//	
	//	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	//	pcl::SACSegmentation<pcl::PointXYZ> seg;
	//	pcl::ExtractIndices<pcl::PointXYZ> extract;
	//	std::vector<ofVec3f> vCloudHull;

	//	// Optional
	//	seg.setOptimizeCoefficients (true);
	//	// Mandatory
	//	seg.setModelType (pcl::SACMODEL_PLANE);
	//	seg.setMethodType (pcl::SAC_RANSAC);
	//	seg.setMaxIterations (50);
	//	seg.setDistanceThreshold (planeTolerance); //original: 0.01
	//	// Create the filtering object
	//	int i = 0, nr_points = cloudTemp->points.size ();
	//	// mientras 7% de la nube no se haya procesado

	//	vector<PCPolygonPtr> nuevos;

	//	int numFaces = 0;
	//	
	//	while (cloudTemp->points.size () > 0.07 * nr_points && numFaces < maxFaces)
	//	{
	//		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	//		// Segment the largest planar component from the remaining cloud
	//		seg.setInputCloud (cloudTemp);
	//		seg.segment (*inliers, *coefficients);
	//		if (inliers->indices.size () == 0) {
	//			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	//			break;
	//		}

	//		//FIX
	//		PCPtr cloud_filtered_temp_inliers (new PC());
	//		PCPtr cloud_filtered_temp_outliers (new PC());
	//		if(inliers->indices.size() != cloudTemp->size())
	//		{
	//			// Extract the inliers
	//			extract.setInputCloud (cloudTemp);
	//			extract.setIndices (inliers);
	//			extract.setNegative (false);
	//			extract.filter (*cloud_filtered_temp_inliers);
	//			cloud_p = cloud_filtered_temp_inliers;
	//		}
	//		else
	//			cloud_p = cloudTemp;
	//	
	//		// Create the filtering object
	//		extract.setInputCloud (cloudTemp);
	//		extract.setIndices (inliers);
	//		extract.setNegative (true);

	//	
	//		if(cloud_p->size() != cloudTemp->size())
	//			extract.filter (*cloud_filtered_temp_outliers);

	//		cloudTemp = cloud_filtered_temp_outliers;

	//		saveCloudAsFile("prefilter_pol" + ofToString(i) + ".pcd",*cloud_p);
	//		
	//		//Remove outliers by clustering
	//		PCPtr cloud_p_filtered (new PC());
	//		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	//		tree->setInputCloud (cloud_p);

	//		std::vector<pcl::PointIndices> cluster_indices;
	//		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	//		ec.setClusterTolerance (0.02); 
	//		ec.setMinClusterSize (5);
	//		ec.setMaxClusterSize (10000);
	//		ec.setSearchMethod (tree);
	//		ec.setInputCloud(cloud_p);

	//		ec.extract (cluster_indices);
	//		int debuccount = 0;

	//		if(cluster_indices.size() > 0)
	//		{
	//			for (std::vector<int>::const_iterator pit = cluster_indices.at(0).indices.begin (); pit != cluster_indices.at(0).indices.end (); pit++)
	//				cloud_p_filtered->points.push_back (cloud_p->points[*pit]); //*
	//		}

	//		saveCloudAsFile("postfilter_pol" + ofToString(i) + ".pcd",*cloud_p_filtered);
	//		if (cloud_p_filtered->size() < 4)
	//			break;

	//		PCPolygonPtr pcp(new PCQuadrilateral(*coefficients, cloud_p_filtered));
	//		pcp->detectPolygon();
	//		pcp->getPolygonModelObject()->setContainer(this);
	//		
	//		nuevos.push_back(pcp);
	//		
	//		i++;
	//		numFaces++;
	//	}
	//	


	//	return nuevos;
	//}

	vector<PCPolygonPtr> PCPolyhedron::detectPolygons(const PCPtr& cloud, float planeTolerance, float pointsTolerance, bool limitFaces)
	{
		PCPtr cloudTemp(cloud);
		
		float maxFaces = limitFaces ? MAX_FACES : 100;
		sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2());
		sensor_msgs::PointCloud2::Ptr cloud_filtered_blob (new sensor_msgs::PointCloud2);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		
		vector<PCPolygonPtr> nuevos;
		
		if(false)
		{
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			std::vector<ofVec3f> vCloudHull;
			
			double angle = 5*(PI/180);
			for(int i = 0; i < pcpolygons.size(); i++)
			{
				PCPolygonPtr pol = pcpolygons.at(i);
				if(!pol->isEstimated())
				{
					PCPtr cloud_p (new PC());
					vector<int> inliersIdx;

					pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>::Ptr seg (new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>(cloudTemp));
					seg->setEpsAngle(angle);
					seg->setInputCloud(cloudTemp);
	
					Eigen::Vector3f axis(pol->getNormal().x,pol->getNormal().y,pol->getNormal().z);
					seg->setAxis(axis);
					pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (seg);
					ransac.setDistanceThreshold (planeTolerance);
					ransac.computeModel();
					ransac.getInliers(inliersIdx);
					ransac.setMaxIterations(50);

					for(int idx = 0; idx < inliersIdx.size(); idx++)
					{
						cloud_p->push_back(cloudTemp->at(inliersIdx.at(idx)));
					}

					saveCloudAsFile("parallelfrom" + ofToString(i) + ".pcd",*pol->getCloud());

					saveCloudAsFile("parallel" + ofToString(i) + ".pcd",*cloud_p);
			
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

					//TODO: tomar mas de un cluster, no solo el primero
					if(cluster_indices.size() > 0)
					{
						for (std::vector<int>::const_iterator pit = cluster_indices.at(0).indices.begin (); pit != cluster_indices.at(0).indices.end (); pit++)
							cloud_p_filtered->points.push_back (cloud_p->points[*pit]); //*
					}

					saveCloudAsFile("postfilter_parallel" + ofToString(i) + ".pcd",*cloud_p_filtered);
					if (cloud_p_filtered->size() < 4)
						break;

					pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
					Eigen::VectorXf coeff;
					ransac.getModelCoefficients(coeff);
					float a = coeff.x();
					float d = coeff.w();
					coefficients->values.push_back(coeff.x());
					coefficients->values.push_back(coeff.y());
					coefficients->values.push_back(coeff.z());
					coefficients->values.push_back(coeff.w());

					PCPolygonPtr pcp(new PCQuadrilateral(*coefficients, cloud_p_filtered));
					pcp->detectPolygon();
					pcp->getPolygonModelObject()->setContainer(this);
			
					nuevos.push_back(pcp);
				}
			}
		}
		else
		{
			PCPtr cloud_p (new PC());
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
			
				nuevos.push_back(pcp);
			
				i++;
				numFaces++;
			}
		}

		return nuevos;
	}

	void PCPolyhedron::namePolygons(vector<PCPolygonPtr>& toName)
	{
		int sideFacesName = 1;
		sort(toName.begin(), toName.end(), xAxisSort);
		for(int i = 0; i < toName.size(); i++)
		{
			if(gModel->table->isParallelToTable(toName.at(i)))
				toName.at(i)->getPolygonModelObject()->setName(kPolygonNameTop);
			else
			{
				toName.at(i)->getPolygonModelObject()->setName((IPolygonName)sideFacesName);
				sideFacesName++;
			}
		}
	}

	void PCPolyhedron::detectPrimitives() {
		vector<PCPolygonPtr> nuevos = detectPolygons(cloud); 
		nuevos = discardPolygonsOutOfBox(nuevos); 
		namePolygons(nuevos);
		vector<PCPolygonPtr> estimated = estimateHiddenPolygons(nuevos);
		pcpolygons = nuevos;
		pcpolygons.insert(pcpolygons.end(),estimated.begin(),estimated.end());		
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
					//Vertex vx;
					ofVec3f avg(0, 0, 0);
					for (int i = 0; i < updateVertexs.size(); i++) {
						avg += updateVertexs.at(i).pcp->getPolygonModelObject()->getVertexs()[updateVertexs.at(i).vertex];
					}
					avg /= updateVertexs.size();
					for (int i = 0; i < updateVertexs.size(); i++) {
						updateVertexs.at(i).pcp->getPolygonModelObject()->setVertex(updateVertexs.at(i).vertex, avg);
						//vx.Polygons.push_back(PCPolygonPtr(updateVertexs.at(i).pcp));
					}
					//vx.setPoint(avg);
					//vertexs.push_back(vx);
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

		//for(int i = 0; i < vertexs.size(); i++)
		//{
		//	cout << "vertex " << ofToString(i) << ": " << vertexs.at(i).getPoint() << endl;
		//	for(int j = 0; j < vertexs.at(i).Polygons.size(); j ++)
		//		cout << "\t" << vertexs.at(i).Polygons.at(j)->getPolygonModelObject()->getName() << endl;
		//}
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

	void PCPolyhedron::increaseLod(const PCPtr& nuCloud) {
		PCModelObject::increaseLod(nuCloud);
		addToModel(nuCloud);
		/*for(vector<PCPolygonPtr>::iterator iter = pcpolygons.begin(); iter != pcpolygons.end(); iter++){
			(*iter)->increaseLod(cloud);
		}
		unifyVertexs();*/
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
	

	vector<PCPolygonPtr> PCPolyhedron::estimateHiddenPolygons(const vector<PCPolygonPtr>& newPolygons)
	{
		return vector<PCPolygonPtr>();
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
		
		if(false) //partialestimation
		{
			nuevos = mergePolygons(nuevos);
			vector<PCPolygonPtr> estimated = estimateHiddenPolygons(nuevos);
			estimated = mergePolygons(estimated);
			nuevos.insert(nuevos.end(),estimated.begin(),estimated.end());
		}
		else
		{
			//Descarto caras que no sean paralelas a la mesa o esten apoyadas sobre la mesa
			cout << "pre discard: " << nuevos.size() << endl;
			nuevos = discardPolygonsOutOfBox(nuevos);
			cout << "post discard: " << nuevos.size() << endl;
			nuevos = mergePolygons(nuevos);
			vector<PCPolygonPtr> estimated = estimateHiddenPolygons(nuevos);
			nuevos.insert(nuevos.end(),estimated.begin(),estimated.end());
		}
		
		pcpolygons = nuevos;
		unifyVertexs();
		
		for(int i = 0; i < pcpolygons.size(); i ++)
		{
			saveCloudAsFile ("pol" + ofToString(pcpolygons.at(i)->getPolygonModelObject()->getName()) + ".pcd", *pcpolygons.at(i)->getCloud());
		}
	}

	void PCPolyhedron::setAndUpdateCloud(const PCPtr& cloud)
	{
		setCloud(cloud);
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
