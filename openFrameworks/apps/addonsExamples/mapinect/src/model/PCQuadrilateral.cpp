#include "PCQuadrilateral.h"

#include "Triangle2D.h"
#include "ofVecUtils.h"
#include "PointUtils.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>

namespace mapinect {

	PCQuadrilateral::PCQuadrilateral(pcl::ModelCoefficients coefficients){
		this->coefficients = coefficients;
	}

	bool PCQuadrilateral::detectPolygon(pcl::PointCloud<PointXYZ>::Ptr cloud, const std::vector<ofVec3f>& vCloud) {
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

	void PCQuadrilateral::increaseLod(PointCloud<PointXYZ>::Ptr nuCloud)
	{
		//PCDWriter writer;
		//writer.write<pcl::PointXYZ> ("nuCloud.pcd", *nuCloud, false);
		//writer.write<pcl::PointXYZ> ("oldCloud.pcd", cloud, false);
		PCDWriter writer;
		//writer.write<pcl::PointXYZ> ("nuCloud.pcd", *nuCloud, false);

		PointCloud<pcl::PointXYZ>::Ptr nuPointsOfFace (new PointCloud<PointXYZ>());
		pcl::PointIndices::Ptr inliers = adjustPlane(coefficients,nuCloud);

		if (inliers->indices.size () == 0) {
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			return;
		}

		//FIX
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

		//writer.write<pcl::PointXYZ> ("nuPointsOfFace.pcd", *nuPointsOfFace, false);
		//writer.write<pcl::PointXYZ> ("oldCloud.pcd", cloud, false);

		this->cloud += *nuPointsOfFace;

		//writer.write<pcl::PointXYZ> ("nucloudOfFace.pcd", cloud, false);
		std::vector<ofVec3f> vCloud;
		for (int k = 0; k < cloud.size(); k++) {
			vCloud.push_back(POINTXYZ_OFXVEC3F(cloud.at(k)));
		}
		pcl::PointCloud<PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<PointXYZ>(cloud));

		matched = new PCQuadrilateral(coefficients);
		matched->detectPolygon(cloudPtr, vCloud);
		updateMatching();
	}

}

