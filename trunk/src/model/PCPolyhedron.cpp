#include "PCPolyhedron.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include "PCQuadrilateral.h"
#include "pointUtils.h"
#include "utils.h"
#include "ofxVecUtils.h"
#include <pcl/io/pcd_io.h>


#define MAX_FACES		3

namespace mapinect {

	PCPolyhedron::PCPolyhedron(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ>::Ptr extendedCloud, int objId)
				: PCModelObject(cloud, extendedCloud, objId)
	{
		drawPointCloud = false; 
		ofxVec3f v;
		Eigen::Vector4f eCenter;
		pcl::compute3DCentroid(*cloud,eCenter);
		
		v.x = eCenter[0];
		v.y = eCenter[1];
		v.z = eCenter[2];

		this->setCenter(v);
	}

	bool hasNoMatching(PCPolygon* p) {
		return !(p->hasMatching());
	}

	void PCPolyhedron::mergePolygons(vector<PCPolygon*> toMerge) {
		vector<PCPolygon*> aAgregar;
		vector<PCPolygon*> aProcesar;

		int remainingIters = 10;
		do {
			for (int i = 0; i < toMerge.size(); i++) {
				PCPolygon*	removed = NULL;
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

		vector<PCPolygon*> keep;
		for (vector<PCPolygon*>::iterator iter = pcpolygons.begin(); iter != pcpolygons.end(); iter++) {
			if (!(*iter)->hasMatching()) {
				delete *iter;				//Puede no tener que borrarse, puede pasar a ser una cara oculta
			}
			else {
				keep.push_back(*iter);
			}
		}
		pcpolygons = keep;

		updatePolygons();

		for (int i = 0; i < aAgregar.size(); i++) {
			if (indexOf(pcpolygons, aAgregar[i]) < 0) {
				pcpolygons.push_back(aAgregar[i]);
			}
		}

		aAgregar.clear();

		cout << "pols: " << pcpolygons.size() << endl;
	}

	vector<PCPolygon*> PCPolyhedron::detectPolygons(PointCloud<pcl::PointXYZ>::Ptr cloudTemp, float planeTolerance, float pointsTolerance, bool limitFaces){
		float maxFaces = limitFaces ? MAX_FACES : 100;
		sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2());
		sensor_msgs::PointCloud2::Ptr cloud_filtered_blob (new sensor_msgs::PointCloud2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		std::vector<ofxVec3f> vCloudHull;

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

		vector<PCPolygon*> nuevos;

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
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_temp_inliers (new pcl::PointCloud<pcl::PointXYZ>());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_temp_outliers (new pcl::PointCloud<pcl::PointXYZ>());
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

			//pcl::io::savePCDFile("prefilter_pol" + ofToString(i) + ".pcd",*cloud_p);
			
			//Remove outliers by clustering
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p_filtered (new pcl::PointCloud<pcl::PointXYZ>());
			pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
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

			//pcl::io::savePCDFile("postfilter_pol" + ofToString(i) + ".pcd",*cloud_p_filtered);

			vCloudHull.clear();
			for (int k = 0; k < cloud_p_filtered->size(); k++) {
				vCloudHull.push_back(POINTXYZ_OFXVEC3F(cloud_p_filtered->at(k)));
			}

			PCPolygon* pcp = new PCQuadrilateral(*coefficients);
			static int polygonId = 0;
			pcp->setId(polygonId++);
			pcp->detectPolygon(cloud_p_filtered, vCloudHull);
			PointCloud<PointXYZ>::Ptr cloud_pTemp (new PointCloud<PointXYZ>(*cloud_p_filtered));
			pcp->setCloud(cloud_pTemp);
			nuevos.push_back(pcp);
			
			//writer.write<pcl::PointXYZ> ("cloud_pTemp" + ofToString(i) + ".pcd", *cloud_pTemp, false);
			i++;
			numFaces++;
		}

		return nuevos;
	}

	void PCPolyhedron::detectPrimitives() {
		PointCloud<PointXYZ>::Ptr cloud_Temp (new PointCloud<PointXYZ>(cloud));
		vector<PCPolygon*> nuevos = detectPolygons(cloud_Temp); 
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

		for (vector<PCPolygon*>::iterator nextIter = pcpolygons.begin(); nextIter != pcpolygons.end();) {
			vector<PCPolygon*>::iterator iter = nextIter++;
			Polygon* polygon = (*iter)->getPolygonModelObject();

			if (polygon == NULL) {
				// No model object is available yet, quit!
				return;
			}

			for (int j = 0; j < polygon->getVertexCount(); j++) {
				updateVertexs.clear();
				VertexInPCPolygon vpp;
				vpp.pcp = *iter;
				vpp.vertex = j;
				updateVertexs.push_back(vpp);
				ofxVec3f v(polygon->getVertex(j));

				for (vector<PCPolygon*>::iterator iter2 = iter; iter2 != pcpolygons.end(); iter2++) {
					Polygon* polygon2 = (*iter2)->getPolygonModelObject();
					for (int k = 0; k < polygon2->getVertexCount(); k++) {
						ofxVec3f v2(polygon2->getVertex(k));
						if (!(v == v2)
							&& polygon->getVertex(j).distance(polygon2->getVertex(k)) <= MAX_UNIFYING_DISTANCE) {
							VertexInPCPolygon vpp2;
							vpp2.pcp = *iter2;
							vpp2.vertex = k;
							updateVertexs.push_back(vpp2);
						}
					}
				}

				if (updateVertexs.size() > 1) {
					ofxVec3f avg(0, 0, 0);
					for (int i = 0; i < updateVertexs.size(); i++) {
						avg += updateVertexs.at(i).pcp->getPolygonModelObject()->getVertex(updateVertexs.at(i).vertex);
					}
					avg /= updateVertexs.size();
					for (int i = 0; i < updateVertexs.size(); i++) {
						updateVertexs.at(i).pcp->getPolygonModelObject()->setVertex(updateVertexs.at(i).vertex, avg);
					}
				}
			}
		}
	}

	bool PCPolyhedron::findBestFit(PCPolygon* polygon, PCPolygon*& removed, bool& wasRemoved)
	{
		for (int i = 0; i < pcpolygons.size(); i++) {
			if (pcpolygons[i]->matches(polygon, removed, wasRemoved)) {
				return true;
			}
		}
		return false;
	}

	void PCPolyhedron::draw() {
		for (vector<PCPolygon*>::iterator iter = pcpolygons.begin(); iter != pcpolygons.end(); iter++) {
			(*iter)->draw();
		}
	}

	void PCPolyhedron::applyTransformation()
	{
		PCModelObject::applyTransformation();
		for(vector<PCPolygon*>::iterator iter = pcpolygons.begin(); iter != pcpolygons.end(); iter++){
			(*iter)->applyTransformation(&transformation);
		}
	}

	PCPolygon*	PCPolyhedron::getPCPolygon(int index)
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

		for(vector<PCPolygon*>::iterator iter = pcpolygons.begin(); iter != pcpolygons.end(); iter++){
			PointCloud<PointXYZ>::Ptr nuCloud (new PointCloud<PointXYZ>(cloud));
			(*iter)->increaseLod(nuCloud);
		}
		unifyVertexs();
	}

	vector<PCPolygon*>	PCPolyhedron::discardPolygonsOutOfBox(vector<PCPolygon*> toDiscard)
	{
		vector<PCPolygon*> polygonsInBox;
		PCPolygon* table = getTable();

		//pcl::io::savePCDFile("table.pcd",table->getCloud());
		
		for(int i = 0; i < toDiscard.size(); i++)
		{
			//pcl::io::savePCDFile("pol" + ofToString(i) + ".pcd",toDiscard.at(i)->getCloud());

			PointCloud<PointXYZ>::Ptr cloudPtr (new PointCloud<PointXYZ>(toDiscard.at(i)->getCloud()));
			if(onTable(cloudPtr,table))
			{
				cout << "pol" << ofToString(i) << " On table!" <<endl;
				polygonsInBox.push_back(toDiscard.at(i));
			}
			else if(tableParallel(toDiscard.at(i), table))
			{
				cout << "pol" << ofToString(i) << " parallel table!" <<endl;
				polygonsInBox.push_back(toDiscard.at(i));
				//pcl::io::savePCDFile("pol" + ofToString(i) + ".pcd",toDiscard.at(i)->getCloud());
			}

		}

		return polygonsInBox;
	}

	void PCPolyhedron::addToModel(PointCloud<PointXYZ>::Ptr nuCloud)
	{
		PCModelObject::addToModel(nuCloud);
		PointCloud<PointXYZ>::Ptr cloudPtr (new PointCloud<PointXYZ>(cloud));

		///TODO:
		/// 1 - Detectar caras de la nueva nube
		/// 2 - Descartar caras que no pertenecen a la caja (caras de la mano)
		/// 3 - Matchear caras de la nube anterior con las nuevas caras
		/// 4 - Estimar caras ocultas (?)
		
		//Detecto nuevas caras
		vector<PCPolygon*> nuevos = detectPolygons(nuCloud,0.003,2.6,false); 

		cout << "pre discard: " << nuevos.size() << endl;
		//Descarto caras fuera del objeto
		vector<PCPolygon*> inBox = discardPolygonsOutOfBox(nuevos);
		
		cout << "post discard: " << inBox.size() << endl;

		mergePolygons(inBox);
		unifyVertexs();


		//*nuCloud += *cloudPtr;
		//for(vector<PCPolygon*>::iterator iter = pcpolygons.begin(); iter != pcpolygons.end(); iter++){
		//	//(*iter)->applyTransformation(&transformation);
		//	(*iter)->increaseLod(nuCloud);
		//}
		//unifyVertexs();
		//pcl::io::savePCDFileASCII ("merged.pcd", *nuCloud);

	}
}
