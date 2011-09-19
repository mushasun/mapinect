#include "Object3D.h"
#include "pointUtils.h"


Object3D::Object3D(){};
Object3D::Object3D(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ>::Ptr extendedCloud, ofxKinect *kinect)
{
	this->cloud = PointCloud<PointXYZ>(*cloud);
	this->extendedcloud = PointCloud<PointXYZ>(*extendedCloud);
	this->kinect = kinect;
	//PointCloud<pcl::PointXYZ>::Ptr cloudTemp (new PointCloud<PointXYZ>(*cloud));
	findPointCloudBoundingBox(cloud, vMin, vMax);
	transformation.setIdentity();
	detectFaces();
}

void Object3D::detectFaces(){
	sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2());
	sensor_msgs::PointCloud2::Ptr cloud_filtered_blob (new sensor_msgs::PointCloud2);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>)
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	std::vector<ofxVec3f> vCloudHull;

	//Remover outliers
	PointCloud<pcl::PointXYZ>::Ptr cloudTemp (new PointCloud<PointXYZ>(cloud));
	//sor.setInputCloud (cloudTemp);
	//sor.setMeanK (10); //Cantidad de vecinos a analizar
	//sor.setStddevMulThresh (1.0);
	//sor.filter (*cloud_filtered);

	//writer.write<pcl::PointXYZ> ("filtered.pcd", *cloud_filtered, false);

	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (50);
	seg.setDistanceThreshold (0.009); //original: 0.01

	// Create the filtering object
	int i = 0, nr_points = cloudTemp->points.size ();
	// mientras 10% de la nube no se haya procesado
	numFaces = 0;
	while (cloudTemp->points.size () > 0.1 * nr_points && numFaces < MAX_FACES)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloudTemp);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud (cloudTemp);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_p);
		// Create the filtering object
		extract.setInputCloud (cloudTemp);
		extract.setIndices (inliers);
		extract.setNegative (true);

		//FIX
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_temp (new pcl::PointCloud<pcl::PointXYZ>());
		if(cloud_p->size() != cloudTemp->size())
			extract.filter (*cloud_filtered_temp);
		
		cloudTemp = cloud_filtered_temp;
		
		
		vCloudHull.clear();
		for (int k = 0; k < cloud_p->size(); k++) {
			vCloudHull.push_back(POINTXYZ_OFXVEC3F(cloud_p->at(k)));
		}

		faces[i].findQuad(vCloudHull);
		//detectedPlane.avgNormal = normalEstimation(cloud_p);
		PointCloud<PointXYZ>::Ptr cloud_pTemp (new PointCloud<PointXYZ>(*cloud_p));
		faces[i].cloudQuad = cloud_pTemp;
	
		i++;
		numFaces++;
	}
}

void Object3D::draw(){
	for(int i = 0; i < numFaces; i++){
		Quad3D face = faces[i];
		ofSetColor(0,0,25*i);
		ofxVec3f v;
		glBegin(GL_POINTS);
		for(int j = 0; j < face.cloudQuad->size(); j++){
			v = kinect->getScreenCoordsFromWorldCoords(POINTXYZ_OFXVEC3F(face.cloudQuad->at(j)));
			glVertex3f(v.x, v.y, 0);
		}
		glEnd();

		ofxVec3f v1,v2,v3,v4;
		v1 = POINTXYZ_OFXVEC3F(face.cloudQuad->at(face.getVertexIdxs()->indices.at(0)));
		v1 = kinect->getScreenCoordsFromWorldCoords(v1);
		v2 = POINTXYZ_OFXVEC3F(face.cloudQuad->at(face.getVertexIdxs()->indices.at(1)));
		v2 = kinect->getScreenCoordsFromWorldCoords(v2);
		v3 = POINTXYZ_OFXVEC3F(face.cloudQuad->at(face.getVertexIdxs()->indices.at(2)));
		v3 = kinect->getScreenCoordsFromWorldCoords(v3);
		v4 = POINTXYZ_OFXVEC3F(face.cloudQuad->at(face.getVertexIdxs()->indices.at(3)));
		v4 = kinect->getScreenCoordsFromWorldCoords(v4);

		ofSetColor(0,255,25*i);
		ofCircle(v1.x,v1.y, 5);
		ofCircle(v2.x,v2.y, 5);
		ofCircle(v3.x,v3.y, 5);
		ofCircle(v4.x,v4.y, 5);
	}
}

void Object3D::updateCloud (PointCloud<PointXYZ>::Ptr nuCloud){
	cloud += (*nuCloud);
	detectFaces();
}

Object3D::~Object3D(void)
{
}
