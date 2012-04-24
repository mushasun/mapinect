#include "PCQuadrilateral.h"

#include "Triangle2D.h"
#include "ofxVecUtils.h"
#include "PointUtils.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>

namespace mapinect {

	PCQuadrilateral::PCQuadrilateral(pcl::ModelCoefficients coefficients){
		this->coefficients = coefficients;
	}

	bool PCQuadrilateral::detectPolygon(pcl::PointCloud<PointXYZ>::Ptr cloud, const std::vector<ofxVec3f>& vCloud) {
		//ofxVec3f vMin, vMax;
		findOfxVec3fBoundingBox(vCloud, vMin, vMax);
		ofxVec3f center = vMin + vMax;
		center *= 0.5f;

		//Eigen::Vector4f clusterCentroid;
		//compute3DCentroid(*cloud,clusterCentroid);
		//ofxVec3f center(clusterCentroid.x(), clusterCentroid.y(), clusterCentroid.z());

		int ixA = 0;
		ofxVec3f vA(vCloud.at(ixA));
		for (int k = 1; k < vCloud.size(); k++) {
			ofxVec3f v(vCloud.at(k));
			if (center.distance(v) > center.distance(vA)) {
				ixA = k;
				vA = v;
			}
		}

		int ixB = 0;
		ofxVec3f vB(vCloud.at(ixB));
		for (int k = 1; k < vCloud.size(); k++) {
			ofxVec3f v(vCloud.at(k));
			if (vA.distance(v) > vA.distance(vB)) {
				ixB = k;
				vB = v;
			}
		}

		DiscardCoordinate discard = calculateDiscardCoordinate(vMin, vMax);

		ofxVec2f v2A = discardCoordinateOfxVec3f(vA, discard);
		ofxVec2f v2B = discardCoordinateOfxVec3f(vB, discard);
		mapinect::Line2D lineAB(v2A, v2B);
		int ixC = 0;
		ofxVec2f v2C(v2A);
		double distanceC = 0;
		for (int k = 0; k < vCloud.size(); k++) {
			ofxVec3f v(vCloud.at(k));
			ofxVec2f v2 = discardCoordinateOfxVec3f(v, discard);
			double distance = lineAB.distance(v2);
			if (distance > distanceC) {
				distanceC = distance;
				ixC = k;
				v2C = v2;
			}
		}

		//cout << "max distance to line: " << distanceC << endl;

		
		int ixD = ixC;
		ofxVec2f v2D(v2C);
		mapinect::Triangle2D triangleABC(v2A, v2B, v2C);
		PositionToLine ptlCtoAB = lineAB.positionTo(v2C);
		double distanceD = 0;
		for (int k = 0; k < vCloud.size(); k++) {
			ofxVec3f v(vCloud.at(k));
			ofxVec2f v2 = discardCoordinateOfxVec3f(v, discard);
			double distance = triangleABC.distance(v2);
			//double distance = lineAB.distance(v2);
			//if (lineAB.positionTo(v2) != ptlCtoAB && distance > distanceD) {
			if (distance > distanceD) {
				distanceD = distance;
				ixD = k;
				v2D = v2;
			}
		}
		

		ofxVec3f v3A(vCloud.at(ixA));
		ofxVec3f v3B(vCloud.at(ixB));
		ofxVec3f v3C(vCloud.at(ixC));
		//ofxVec3f v3Center = (v3A + v3B) * 0.5;
		//ofxVec3f v3D = v3Center + (v3Center - v3C);
		ofxVec3f v3D(vCloud.at(ixD));

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

	void PCQuadrilateral::increaseLod(PointCloud<PointXYZ>::Ptr nuCloud)
	{
		PCDWriter writer;
		//writer.write<pcl::PointXYZ> ("nuCloudFace"+ ofToString(this->getId())+".pcd", *nuCloud, false);
		//writer.write<pcl::PointXYZ> ("oldCloud.pcd", cloud, false);
		//writer.write<pcl::PointXYZ> ("nuCloud.pcd", *nuCloud, false);

		PointCloud<pcl::PointXYZ>::Ptr nuPointsOfFace (new PointCloud<PointXYZ>());

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
		//ofxVec3f center = this->getCenter();
		//Eigen::Vector4f partialCenter;
		//float mindist = numeric_limits<float>::max();
		//pcl::PointCloud<pcl::PointXYZ>::Ptr closest (new pcl::PointCloud<pcl::PointXYZ>());

		////Tomo el cluster más cercano al actual
		//if(cluster_indices.size() > 0)
		//{
		//	for(int i = 0; i < cluster_indices.size(); i++)
		//	{
		//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p_filtered (new pcl::PointCloud<pcl::PointXYZ>());
		//		cloud_p_filtered->resize(cluster_indices.at(i).indices.size());
		//		for (std::vector<int>::const_iterator pit = cluster_indices.at(i).indices.begin (); pit != cluster_indices.at(i).indices.end (); pit++)
		//			cloud_p_filtered->points.push_back (nuPointsOfFace->points[*pit]); //*
		//		pcl::compute3DCentroid(*cloud_p_filtered,partialCenter);
		//		ofxVec3f vPartialCenter(partialCenter.x(),partialCenter.y(),partialCenter.z());

		//		if(abs((center - vPartialCenter).length()) < mindist)
		//		{
		//			mindist = abs((center - vPartialCenter).length());
		//			closest = cloud_p_filtered;
		//		}
		//	}
		//}
		
		//writer.write<pcl::PointXYZ> ("oldCloud.pcd", cloud, false);
		//merge de nubes
		this->cloud += *nuPointsOfFace;

		//writer.write<pcl::PointXYZ> ("nucloudOfFace"+ ofToString(this->getId())+".pcd", cloud, false);
		std::vector<ofxVec3f> vCloud;
		for (int k = 0; k < cloud.size(); k++) {
			vCloud.push_back(POINTXYZ_OFXVEC3F(cloud.at(k)));
		}
		pcl::PointCloud<PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<PointXYZ>(cloud));

		matched = new PCQuadrilateral(coefficients);
		matched->detectPolygon(cloudPtr, vCloud);
		updateMatching();
	}

}

