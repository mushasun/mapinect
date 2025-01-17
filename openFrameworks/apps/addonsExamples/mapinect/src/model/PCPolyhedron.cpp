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
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>


#include "ofUtils.h"

#include "Constants.h"
#include "DataObject.h"
#include "Globals.h"
#include "ofVecUtils.h"
#include "PCQuadrilateral.h"
#include "pointUtils.h"
#include "transformationUtils.h"
#include "utils.h"
#include "Feature.h"


#define MAX_FACES		3

namespace mapinect {

	PCPolyhedron::PCPolyhedron(const PCPtr& cloud, int objId)
				: PCModelObject(cloud, objId)
	{
		fullEstimation = false;
	}

	IObjectPtr PCPolyhedron::getMathModelApproximation() const
	{
		pcPolygonsMutex.lock();
		vector<IPolygonPtr> polygons;
		for (vector<PCPolygonPtr>::const_iterator p = pcpolygons.begin(); p != pcpolygons.end(); ++p)
		{
			polygons.push_back((*p)->getMathPolygonModelApproximation());
		}
		IObjectPtr result(new DataObject(getId(), getCenter(), getScale(), getRotation(), polygons));
		for (vector<IPolygonPtr>::iterator p = polygons.begin(); p != polygons.end(); ++p)
		{
			dynamic_cast<Polygon*>(p->get())->setContainer(result);
		}
		pcPolygonsMutex.unlock();
		return result;
	}

	const vector<IPolygonPtr> PCPolyhedron::getVisiblePolygons()
	{
		vector<IPolygonPtr> pols;
		for (vector<PCPolygonPtr>::iterator p = pcpolygons.begin(); p != pcpolygons.end(); ++p)
		{
			if(!(*p)->isEstimated())
				pols.push_back((*p)->getMathPolygonModelApproximation());
		}
		return pols;
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
		
		vector<PCPolygonPtr> missing = updatePolygons();

		vector<PCPolygonPtr> polygonsInBox(discardPolygonsOutOfBox(aAgregar, keep));

		// Setea el nombre de los poligonos nuevos, con el poligono m�s cercano
		for (int i = 0; i < polygonsInBox.size(); i++) {
			if (indexOf(pcpolygons, polygonsInBox[i]) < 0) {
				PCPolygonPtr closer = findCloserPolygon(polygonsInBox[i],missing);
				polygonsInBox[i]->getPolygonModelObject()->setName(closer->getPolygonModelObject()->getName());
				polygonsInBox[i]->setEstimated(false);
				keep.push_back(polygonsInBox[i]);
			}
		}

		polygonsInBox.clear();

		return keep;
	}
	
	vector<PCPolygonPtr> PCPolyhedron::detectPolygons(const PCPtr& cloud, float planeTolerance, float pointsTolerance, bool limitFaces)
	{
		float DOT_EPSILON = 0.15;
		saveCloud("1_toDetect.pcd", *cloud);

		PCPtr cloudTemp(cloud);
		
		float maxFaces = limitFaces ? MAX_FACES : 100;
		sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2());
		sensor_msgs::PointCloud2::Ptr cloud_filtered_blob (new sensor_msgs::PointCloud2);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		
		vector<PCPolygonPtr> nuevos;
		
		PCPtr cloudP (new PC());
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
		int i = 0, nrPoints = cloudTemp->points.size ();
		// mientras 7% de la nube no se haya procesad+o

		int numFaces = 0;
		
		while (cloudTemp->points.size () > 0.07 * nrPoints && numFaces < maxFaces)
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
			PCPtr cloudFilteredTempInliers (new PC());
			PCPtr cloudFilteredTempOutliers (new PC());
			if(inliers->indices.size() != cloudTemp->size())
			{
				// Extract the inliers
				extract.setInputCloud (cloudTemp);
				extract.setIndices (inliers);
				extract.setNegative (false);
				extract.filter (*cloudFilteredTempInliers);
				cloudP = cloudFilteredTempInliers;
			}
			else
				cloudP = cloudTemp;
		
			// Create the filtering object
			extract.setInputCloud (cloudTemp);
			extract.setIndices (inliers);
			extract.setNegative (true);

		
			if(cloudP->size() != cloudTemp->size())
				extract.filter (*cloudFilteredTempOutliers);

			cloudTemp = cloudFilteredTempOutliers;

			saveCloud("2_DetectedPol" + ofToString(i) + ".pcd", *cloudP);
			
			//Remove outliers by clustering	
			vector<pcl::PointIndices> clusterIndices(findClusters(cloudP, pointsTolerance, 10, 10000));
			int debuccount = 0;

			PCPtr cloudPFiltered (new PC());
			if(clusterIndices.size() > 0)
			{
				cloudPFiltered = getCloudFromIndices(cloudP, clusterIndices.at(0));
			}

			saveCloud("3_Postfilter_pol" + ofToString(i) + ".pcd",*cloudPFiltered);
			if (cloudPFiltered->size() < 4)
				break;

			//Controlo que las normales sean perpendiculares
			ofVec3f norm (coefficients->values[0],coefficients->values[1],coefficients->values[2]);
			norm.normalize();
			bool normalCheck = true;
			for(int i = 0; i < nuevos.size() && normalCheck; i++)
			{
				float dot = abs(nuevos[i]->getNormal().dot(norm));
				if( dot > DOT_EPSILON)
				{
					normalCheck = false;
				}
			}

			if(normalCheck)
			{
				//proyecto los puntos sobre el plano
				pcl::ProjectInliers<pcl::PointXYZ> proj; 
				proj.setModelType(pcl::SACMODEL_PLANE); 
				PCPtr projectedCloud (new PC()); 
				proj.setInputCloud(cloudPFiltered); 
				proj.setModelCoefficients(coefficients); 
				proj.filter(*projectedCloud);

				saveCloud("4_Proy_Pol" + ofToString(i) + ".pcd",*projectedCloud);

				PCPolygonPtr pcp(new PCQuadrilateral(*coefficients, projectedCloud));
				pcp->detectPolygon();
			
				nuevos.push_back(pcp);
				numFaces++;
			}
			i++;
		}

		return nuevos;
	}

	SORT_ON_PROP(T, CenterXAsc, get()->getCenter().x, <)

	void PCPolyhedron::namePolygons(vector<PCPolygonPtr>& toName)
	{
		int sideFacesName = 1;
		sort(toName.begin(), toName.end(), sortOnCenterXAsc<PCPolygonPtr>);
		for(int i = 0; i < toName.size(); i++)
		{
			gModel->tableMutex.lock();
			if(gModel->getTable()->isParallelToTable(toName.at(i)))
				toName.at(i)->getPolygonModelObject()->setName(kPolygonNameTop);
			else
			{
				toName.at(i)->getPolygonModelObject()->setName((IPolygonName)sideFacesName);
				sideFacesName++;
			}
			gModel->tableMutex.unlock();
		}
	}

	void PCPolyhedron::detectPrimitives() {
		pcPolygonsMutex.lock();
		saveCloud("detectPrimitives.pcd", *cloud);

		vector<PCPolygonPtr> nuevos = detectPolygons(cloud,Constants::OBJECT_PLANE_TOLERANCE(),Constants::CLOUD_VOXEL_SIZE * 2); 
		nuevos = discardPolygonsOutOfBox(nuevos); 
		namePolygons(nuevos);
		bool estimationOk;
		vector<PCPolygonPtr> estimated = estimateHiddenPolygons(nuevos,estimationOk);

		if(estimationOk)
		{
			pcpolygons = nuevos;
			pcpolygons.insert(pcpolygons.end(),estimated.begin(),estimated.end());		
			unifyVertexs();

			polygonsCache.clear();
			for (vector<PCPolygonPtr>::iterator p = pcpolygons.begin(); p != pcpolygons.end(); ++p)
			{
				polygonsCache.push_back((*p)->getPolygonModelObject());
			}

			for(int i = 0; i < pcpolygons.size(); i ++)
			{
				saveCloud("pol" + ofToString(pcpolygons.at(i)->getPolygonModelObject()->getName()) + ".pcd", *pcpolygons.at(i)->getCloud());
			}
		}
		else
			cout << "estimation failed in detect primitives" << endl;
		pcPolygonsMutex.unlock();

	}

	vector<PCPolygonPtr> PCPolyhedron::updatePolygons() {
		vector<PCPolygonPtr> missing;
		for (int i = 0; i < pcpolygons.size(); i++) {
			if(pcpolygons[i]->hasMatching())
			{
				pcpolygons[i]->updateMatching();
				pcpolygons[i]->removeMatching();
			}
			else
				missing.push_back(pcpolygons[i]);
		}
		return missing;
	}

	void PCPolyhedron::unifyVertexs() {
		vertexs.clear();

		struct VertexInPCPolygon
		{
			VertexInPCPolygon(PCPolygon* pcp, int vertex) : pcp(pcp), vertex(vertex) { }
			PCPolygon*		pcp;
			int				vertex;
		};
		vector<VertexInPCPolygon> updateVertexs;

		for (vector<PCPolygonPtr>::iterator nextIter = pcpolygons.begin(); nextIter != pcpolygons.end();) {
			vector<PCPolygonPtr>::iterator iter = nextIter++;
			Polygon* polygon = (*iter)->getPolygonModelObject();

			if (polygon == NULL) {
				// No model object is available yet, quit!
				return;
			}

			for (int j = 0; j < polygon->getMathModel().getVertexs().size(); j++) {
				updateVertexs.clear();
				VertexInPCPolygon vpp(iter->get(), j);
				updateVertexs.push_back(vpp);
				ofVec3f v(polygon->getMathModel().getVertexs()[j]);

				for (vector<PCPolygonPtr>::iterator iter2 = iter; iter2 != pcpolygons.end(); iter2++) {
					Polygon* polygon2 = (*iter2)->getPolygonModelObject();
					for (int k = 0; k < polygon2->getMathModel().getVertexs().size(); k++) {
						ofVec3f v2(polygon2->getMathModel().getVertexs()[k]);
						if (!(v == v2)
							&& polygon->getMathModel().getVertexs()[j].distance(polygon2->getMathModel().getVertexs()[k]) <= Constants::OBJECT_VERTEX_UNIFYING_DISTANCE()) {
							VertexInPCPolygon vpp2(iter2->get(), k);
							updateVertexs.push_back(vpp2);
						}
					}
				}

				if (updateVertexs.size() > 1) {
					ofVec3f avg(0, 0, 0);
					for (int i = 0; i < updateVertexs.size(); i++) {
						avg += updateVertexs.at(i).pcp->getPolygonModelObject()->getMathModel().getVertexs()[updateVertexs.at(i).vertex];
					}
					avg /= updateVertexs.size();
					for (int i = 0; i < updateVertexs.size(); i++) {
						updateVertexs.at(i).pcp->getPolygonModelObject()->setVertex(updateVertexs.at(i).vertex, avg);
						//vx.Polygons.push_back(PCPolygonPtr(updateVertexs.at(i).pcp));
					}
					this->vertexs.push_back(avg);
					//vx.setPoint(avg);
					//vertexs.push_back(vx);
				}
			}
		}
			saveCloud("UnifiedVertex.pcd", vertexs);

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

	PCPolygonPtr PCPolyhedron::findCloserPolygon(PCPolygonPtr pol, vector<PCPolygonPtr> polygons)
	{
		PCPolygonPtr closer = pol;
		float dist = numeric_limits<float>::max();
		for(int i = 0; i < polygons.size(); i++)
		{
			float d = polygons.at(i)->getCenter().squareDistance(pol->getCenter());
			if(d < dist)
			{
				closer = polygons.at(i);
				dist = d;
			}
		}
		return closer;
	}

	void PCPolyhedron::draw() {
		pcPolygonsMutex.lock();
		for (int i = 0; i < pcpolygons.size(); i++)
			pcpolygons[i]->draw();

		ofSetColor(this->getColor());
		for(int i = 0; i < vertexs.size(); i++)
		{
			ofVec3f w = getScreenCoords(vertexs.at(i));
			ofCircle(w.x, w.y, 5, 4);
		}
		static vector<pair<int, int> > lines;
		if (lines.empty())
		{
			lines.push_back(make_pair(0, 1));
			lines.push_back(make_pair(1, 2));
			lines.push_back(make_pair(2, 3));
			lines.push_back(make_pair(3, 0));
			lines.push_back(make_pair(4, 5));
			lines.push_back(make_pair(5, 6));
			lines.push_back(make_pair(6, 7));
			lines.push_back(make_pair(7, 4));
			lines.push_back(make_pair(0, 4));
			lines.push_back(make_pair(1, 5));
			lines.push_back(make_pair(2, 6));
			lines.push_back(make_pair(3, 7));
		}
		for(int i = 0; i < lines.size(); i++)
		{
			ofVec3f w1 = getScreenCoords(vertexs[lines[i].first]);
			ofVec3f w2 = getScreenCoords(vertexs[lines[i].second]);
			ofLine(w1.x, w1.y, 5, w2.x, w2.y, 5);
		}

		ofSetColor(0,0,255);
		ofVec3f sAvg = getScreenCoords(this->getCenter());
		ofDrawBitmapString(ofToString(this->getId()), sAvg.x, sAvg.y, 0);


		pcPolygonsMutex.unlock();
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
		pcPolygonsMutex.lock();
		PCModelObject::resetLod();
		for (int i = 0; i < pcpolygons.size(); i++) {
			pcpolygons[i]->resetLod();
		}
		pcPolygonsMutex.unlock();
	}

	void PCPolyhedron::increaseLod(const PCPtr& nuCloud) {
		PCModelObject::increaseLod(nuCloud);
		addToModel(nuCloud);
	}

	vector<PCPolygonPtr>	PCPolyhedron::discardPolygonsOutOfBox(const vector<PCPolygonPtr>& toDiscard)
	{
		return toDiscard;
	}
	
	vector<PCPolygonPtr>	PCPolyhedron::discardPolygonsOutOfBox(const vector<PCPolygonPtr>& toDiscard, const vector<PCPolygonPtr>& inPolygon)
	{
		return toDiscard;
	}

	vector<PCPolygonPtr> PCPolyhedron::estimateHiddenPolygons(const vector<PCPolygonPtr>& newPolygons, bool& estimationOk)
	{
		estimationOk = true;
		return vector<PCPolygonPtr>();
	}

	void PCPolyhedron::addToModel(const PCPtr& nuCloud)
	{
		PCModelObject::addToModel(nuCloud);

		/// 1 - Detectar caras de la nueva nube
		/// 2 - Descartar caras que no pertenecen a la caja (caras de la mano)
		/// 3 - Matchear caras de la nube anterior con las nuevas caras
		/// 4 - Estimar caras ocultas (?)
		
		PCPtr trimmedCloud = getHalo(this->getvMin(),this->getvMax(),0.05,nuCloud);
		
		saveCloud("nucloud.pcd",*nuCloud);
		saveCloud("trimmed.pcd",*trimmedCloud);

		pcPolygonsMutex.lock();
		vector<PCPolygonPtr> prevPcPolygons = pcpolygons;

		//Detecto nuevas caras
		vector<PCPolygonPtr> nuevos = detectPolygons(trimmedCloud,Constants::OBJECT_PLANE_TOLERANCE(),Constants::CLOUD_VOXEL_SIZE * 2,false); 
		
		//Matching de las caras detectadas con las anteriores
		nuevos = mergePolygons(nuevos);

		//Estimaci�n de caras no visibles
		bool estimationOK;
		vector<PCPolygonPtr> estimated = estimateHiddenPolygons(nuevos,estimationOK);

		bool valid = true;
		if(estimationOK)
		{
			for(int i = 0; i < nuevos.size(); i++)
				nuevos.at(i)->setEstimated(false);
			//Actualizo los nuevos polygons si no hubo errores
			nuevos.insert(nuevos.end(),estimated.begin(),estimated.end());
			pcpolygons = nuevos;

			//Unifico vertices
			unifyVertexs();

			isvalid = validate();
		}

		//Chequeo volumenes
		if(IsFeatureActive(FEATURE_INVALIDATE_OBJECT_BY_VOLUME))
		{
			float inVol = computeVolume(trimmedCloud);

			if(getVolume() < inVol * Constants::OBJECT_VOLUME_TOLERANCE)
			{
				cout << "[   INVALID VOLUME    ]" << endl;
				cout << "need: " << inVol * 0.6 << "  --- has: " << getVolume() << endl;
				isvalid = false;
			}
		}
		
		if(estimationOK && isvalid)
		{
			polygonsCache.clear();
			for (vector<PCPolygonPtr>::iterator p = pcpolygons.begin(); p != pcpolygons.end(); ++p)
			{
				polygonsCache.push_back((*p)->getPolygonModelObject());
			}

		}
		else
		{
			pcpolygons = prevPcPolygons;

			//Si hay errores en la estimaci�n, rollback de los pcpolygons mergeados
			for(int i = 0; i < pcpolygons.size(); i++)
				pcpolygons.at(i)->rollBackMatching();

			if(!isvalid)
			{
				//Necesito unificar los vertices despues del rollback
				unifyVertexs();
			}
			isvalid = false;
		}
		
		//Seteo nueva nube
		PCPtr finalCloud (new PC());
		for(int i = 0; i < pcpolygons.size(); i ++)
		{
			*finalCloud += *pcpolygons.at(i)->getCloud();
			saveCloud("pol" + ofToString(pcpolygons.at(i)->getPolygonModelObject()->getName()) + ".pcd", *pcpolygons.at(i)->getCloud());
		}
		saveCloud("finalCloud" + ofToString(getId()) + ".pcd", *finalCloud);

		setCloud(finalCloud);
		pcPolygonsMutex.unlock();

	}

	bool PCPolyhedron::isFaceOccluded(IPolygonName name)
	{
		return find(occludedFaces.begin(), occludedFaces.end(),name) != occludedFaces.end();
	}

	bool PCPolyhedron::validate()
	{
		return true;
	}

}
