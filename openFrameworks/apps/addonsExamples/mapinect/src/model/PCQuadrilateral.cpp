#include "PCQuadrilateral.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>

#include "Triangle2D.h"
#include "ofVecUtils.h"
#include "PointUtils.h"

namespace mapinect {

	bool PCQuadrilateral::detectPolygon() {
		vector<ofVec3f> vCloud = pointCloudToOfVecVector(cloud);

		//ofVec3f vMin, vMax;
		findOfxVec3fBoundingBox(vCloud, vMin, vMax);
		ofVec3f center = vMin + vMax;
		center *= 0.5f;

		//Eigen::Vector4f clusterCentroid;
		//compute3DCentroid(*cloud,clusterCentroid);
		//ofVec3f center(clusterCentroid.x(), clusterCentroid.y(), clusterCentroid.z());

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

		DiscardCoordinate discard = calculateDiscardCoordinate(vMin, vMax);

		ofVec2f v2A = discardCoordinateOfxVec3f(vA, discard);
		ofVec2f v2B = discardCoordinateOfxVec3f(vB, discard);
		mapinect::Line2D lineAB(v2A, v2B);
		int ixC = 0;
		ofVec2f v2C(v2A);
		double distanceC = 0;
		for (int k = 0; k < vCloud.size(); k++) {
			ofVec3f v(vCloud.at(k));
			ofVec2f v2 = discardCoordinateOfxVec3f(v, discard);
			double distance = lineAB.distance(v2);
			if (distance > distanceC) {
				distanceC = distance;
				ixC = k;
				v2C = v2;
			}
		}

		//cout << "max distance to line: " << distanceC << endl;

		
		int ixD = ixC;
		ofVec2f v2D(v2C);
		mapinect::Triangle2D triangleABC(v2A, v2B, v2C);
		PositionToLine ptlCtoAB = lineAB.positionTo(v2C);
		double distanceD = 0;
		for (int k = 0; k < vCloud.size(); k++) {
			ofVec3f v(vCloud.at(k));
			ofVec2f v2 = discardCoordinateOfxVec3f(v, discard);
			double distance = triangleABC.distance(v2);
			//double distance = lineAB.distance(v2);
			//if (lineAB.positionTo(v2) != ptlCtoAB && distance > distanceD) {
			if (distance > distanceD) {
				distanceD = distance;
				ixD = k;
				v2D = v2;
			}
		}
		

		ofVec3f v3A(vCloud.at(ixA));
		ofVec3f v3B(vCloud.at(ixB));
		ofVec3f v3C(vCloud.at(ixC));
		//ofVec3f v3Center = (v3A + v3B) * 0.5;
		//ofVec3f v3D = v3Center + (v3Center - v3C);
		ofVec3f v3D(vCloud.at(ixD));

		//cout << "max distance to triangle: " << distanceD << endl;
		getPolygonModelObject()->resetVertex();
		getPolygonModelObject()->addVertex(v3A);
		getPolygonModelObject()->addVertex(v3B);
		getPolygonModelObject()->addVertex(v3C);
		getPolygonModelObject()->addVertex(v3D);
		getPolygonModelObject()->sortVertexs();

		std::vector<int> indices (4);
		indices[0] = ixA;
		indices[1] = ixB;
		indices[2] = ixC;
		indices[3] = 0;
		pcl::PointIndices::Ptr v (new pcl::PointIndices ());
		vertexIdxs = v;
		vertexIdxs->indices = indices;


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

