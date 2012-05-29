#include "PCQuadrilateral.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>

#include "Polygon3D.h"
#include "ofVecUtils.h"
#include "PointUtils.h"
#include "ofUtils.h"
#include "Feature.h"

namespace mapinect {

	bool PCQuadrilateral::detectPolygon() {
		if(IsFeatureActive(FEATURE_RECTANGLE_VERTEX))
		{
			vector<ofVec3f> vertexs = findRectangle(cloud, coefficients);
			
			{
				Polygon* p = getPolygonModelObject();
				p->setVertexs(vertexs);
				p->sortVertexs();
				p->setCenter(computeCentroid(cloud));
			}

			saveCloudAsFile("vertex" + ofToString(this->getId()) + ".pcd", vertexs); 
			saveCloudAsFile("vertexCloud" + ofToString(this->getId()) + ".pcd", *cloud); 
		}
		else
		{
			vector<ofVec3f> vCloud = pointCloudToOfVecVector(cloud);

			findOfVec3fBoundingBox(vCloud, vMin, vMax);
			ofVec3f center = computeCentroid(vCloud);

			int ixA = 0;
			ofVec3f vA(vCloud.at(ixA));
			for (int k = 1; k < vCloud.size(); k++) {
				ofVec3f v(vCloud.at(k));
				if (center.distance(v) > center.distance(vA)) {
					ixA = k;
					vA = v;
				}
			}

			int ixB = 0;
			ofVec3f vB(vCloud.at(ixB));
			for (int k = 1; k < vCloud.size(); k++) {
				ofVec3f v(vCloud.at(k));
				if (vA.distance(v) > vA.distance(vB)) {
					ixB = k;
					vB = v;
				}
			}

			DiscardCoordinate discard = calculateDiscardCoordinate(vCloud);

			ofVec3f v3A(vCloud.at(ixA));
			ofVec3f v3B(vCloud.at(ixB));
			mapinect::Line3D lineAB(v3A, v3B);
			int ixC = 0;
			double distanceC = 0;
			for (int k = 0; k < vCloud.size(); k++) {
				ofVec3f v(vCloud.at(k));
				double distance = lineAB.distance(v);
				if (distance > distanceC) {
					distanceC = distance;
					ixC = k;
				}
			}

			//cout << "max distance to line: " << distanceC << endl;

		
			ofVec3f v3C(vCloud.at(ixC));

			int ixD = ixC;
			vector<ofVec3f> vertexs;
			vertexs.push_back(v3A);
			vertexs.push_back(v3B);
			vertexs.push_back(v3C);
			mapinect::Polygon3D triangleABC(vertexs);
			double distanceD = 0;
			for (int k = 0; k < vCloud.size(); k++) {
				double distance = triangleABC.distance(vCloud.at(k));
				if (distance > distanceD) {
					distanceD = distance;
					ixD = k;
				}
			}
		
			ofVec3f v3D(vCloud.at(ixD));
			vertexs.push_back(v3D);

			//cout << "max distance to triangle: " << distanceD << endl;
			{
				Polygon* p = getPolygonModelObject();
				p->setVertexs(vertexs);
				p->sortVertexs();
				p->setCenter(computeCentroid(cloud));
			}

			std::vector<int> indices (4);
			indices[0] = ixA;
			indices[1] = ixB;
			indices[2] = ixC;
			indices[3] = 0;
			pcl::PointIndices::Ptr v (new pcl::PointIndices ());
			vertexIdxs = v;
			vertexIdxs->indices = indices;

			saveCloudAsFile("vertex" + ofToString(this->getId()) + ".pcd", vertexs); 
			saveCloudAsFile("vertexCloud" + ofToString(this->getId()) + ".pcd", *cloud); 
		}
		//cout << "x: " << pVA.x << " y: " << pVA.y << " z: " << pVA.z << endl;
		return true;
	}

	void PCQuadrilateral::increaseLod(const PCPtr& nuCloud)
	{
		pcl::PCDWriter writer;
		//writer.write<pcl::PointXYZ> ("nuCloudFace"+ ofToString(this->getId())+".pcd", *nuCloud, false);
		//writer.write<pcl::PointXYZ> ("oldCloud.pcd", cloud, false);
		//writer.write<pcl::PointXYZ> ("nuCloud.pcd", *nuCloud, false);

		PCPtr nuPointsOfFace (new PC());

		//Obtengo los puntos que pertenecen al plano
		pcl::PointIndices::Ptr inliers = adjustPlane(coefficients,nuCloud);
		if (inliers->indices.size () == 0) {
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			return;
		}
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		if(inliers->indices.size() != nuCloud->size())
		{
			// Extract the inliers
			extract.setInputCloud (nuCloud);
			extract.setIndices (inliers);
			extract.filter (*nuPointsOfFace);
		}
		else
			nuPointsOfFace = nuCloud;

		
		//writer.write<pcl::PointXYZ> ("nuPointsOfFace" + ofToString(this->getId()) + ".pcd", *nuPointsOfFace, false);
		
		//Comentado para commit
		////Elimino outliers
		////Clustering de los puntos del plano
		//pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
		//tree->setInputCloud (nuPointsOfFace);
		//std::vector<pcl::PointIndices> cluster_indices;
		//pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		//ec.setClusterTolerance (0.02); 
		//ec.setMinClusterSize (5);
		//ec.setMaxClusterSize (10000);
		//ec.setSearchMethod (tree);
		//ec.setInputCloud(nuPointsOfFace);

		//ec.extract (cluster_indices);
		//int debuccount = 0;
		//ofVec3f center = this->getCenter();
		//Eigen::Vector4f partialCenter;
		//float mindist = numeric_limits<float>::max();
		//PCPtr closest (new pcl::PointCloud<pcl::PointXYZ>());

		////Tomo el cluster más cercano al actual
		//if(cluster_indices.size() > 0)
		//{
		//	for(int i = 0; i < cluster_indices.size(); i++)
		//	{
		//		PCPtr cloud_p_filtered (new pcl::PointCloud<pcl::PointXYZ>());
		//		cloud_p_filtered->resize(cluster_indices.at(i).indices.size());
		//		for (std::vector<int>::const_iterator pit = cluster_indices.at(i).indices.begin (); pit != cluster_indices.at(i).indices.end (); pit++)
		//			cloud_p_filtered->points.push_back (nuPointsOfFace->points[*pit]); //*
		//		pcl::compute3DCentroid(*cloud_p_filtered,partialCenter);
		//		ofVec3f vPartialCenter(partialCenter.x(),partialCenter.y(),partialCenter.z());

		//		if(abs((center - vPartialCenter).length()) < mindist)
		//		{
		//			mindist = abs((center - vPartialCenter).length());
		//			closest = cloud_p_filtered;
		//		}
		//	}
		//}
		
		//writer.write<pcl::PointXYZ> ("oldCloud.pcd", cloud, false);
		//merge de nubes
		this->cloud->operator+=(*(nuPointsOfFace.get()));

		//writer.write<pcl::PointXYZ> ("nucloudOfFace"+ ofToString(this->getId())+".pcd", cloud, false);
		matched = PCPolygonPtr(new PCQuadrilateral(coefficients, cloud));
		matched->detectPolygon();
		updateMatching();
	}

}

