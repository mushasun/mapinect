#include "PCM.h"

#include "pointUtils.h"
#include "ofxVecUtils.h"
#include "Line2D.h"
#include "Triangle2D.h"
#include "ofGraphicsUtils.h"

#define DEFAULT_NAME		"test"

using namespace std;

//--------------------------------------------------------------
void PCM::setup(ofxKinect *kinect, OpenCV *openCVService) {
	//Cargo archivo de config.
	ofxXmlSettings XML;
	if(XML.loadFile("PCM_Config.xml")){
	
		OCTREE_RES = XML.getValue("PCMConfig:OCTREE_RES", 0.1);
		MIN_DIFF_TO_PROCESS = XML.getValue("PCMConfig:MIN_DIFF_TO_PROCESS", 40);
		QUAD_HALO = XML.getValue("PCMConfig:QUAD_HALO", 10);
		DIFF_THRESHOLD = XML.getValue("PCMConfig:DIFF_THRESHOLD", 20);
		RES_IN_OBJ = XML.getValue("PCMConfig:RES_IN_OBJ", 0.001);
		DIFF_IN_OBJ = XML.getValue("PCMConfig:DIFF_IN_OBJ", 20);
		TIMES_TO_CREATE_OBJ = XML.getValue("PCMConfig:TIMES_TO_CREATE_OBJ", 5);
	}

	this->kinect = kinect;
	this->openCVService = openCVService;
	// Initialize point cloud
	cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
		(new pcl::PointCloud<pcl::PointXYZ>()); 
	cloud->width    = KINECT_WIDTH;
	cloud->height   = KINECT_HEIGHT;
	cloud->is_dense = false;
	cloud->points.resize (CLOUD_POINTS);

	// Inicializo octree
	octree = new octree::OctreePointCloudChangeDetector<PointXYZ>(OCTREE_RES); //Valor de resolucion sacado del ejemplo

	timer = 0;
	baseCloudSetted = false;
	drawPC = false;
	transformation.setIdentity();
	noDifferencesCount = 0;
	nDetectedObjects = 0;
	detectMode = false;

	for(int i = 0; i < MAX_OBJECTS; i++)
		tempObjectsStatus[i] = 0;
}

//--------------------------------------------------------------
void PCM::update(bool isKinectFrameNew) {
	if(isKinectFrameNew)
	{
		if(!baseCloudSetted)
		{
			setInitialPointCloud();
			baseCloudSetted = true;
			cout << "Base cloud setted..." << endl;
		}
		else if(ofGetElapsedTimef() - timer > 0.2) {
			//cout << "Process diferences" << endl;
			if(detectMode)
				processDiferencesClouds();
			timer = ofGetElapsedTimef();
		}
	}
}

//--------------------------------------------------------------
void PCM::draw() {
	ofResetColor();

	if(drawPC) {
		ofPushMatrix();
		ofTranslate(420, 320);
		// we need a proper camera class
		drawPointCloud();
		ofPopMatrix();
	}
	else {
		kinect->drawDepth(0, 0, 640, 480);
		ofResetColor();
		ofPushMatrix();
		/*int w_2 = 640 / 2;
		int h_2 = 480 / 2;
		ofTranslate(420 + w_2, 10 + h_2);
		*///kinect->draw(-w_2, -h_2);

		/*ofEnableAlphaBlending();
		ofSetColor(255,255,255,128);
		kinect->drawDepth(-w_2, -h_2);
		ofDisableAlphaBlending();
*/
		/*ofSetColor(kRGBGreen);
		ofTriangle(detectedPlane.getVA().x, detectedPlane.getVA().y,
			detectedPlane.getVB().x, detectedPlane.getVB().y,
			detectedPlane.getVC().x, detectedPlane.getVC().y);
		ofTriangle(detectedPlane.getVA().x, detectedPlane.getVA().y,
			detectedPlane.getVB().x, detectedPlane.getVB().y,
			detectedPlane.getVD().x, detectedPlane.getVD().y);*/
		
		//detectedPlane.applyTransformation(transformation);
		//glMultMatrixf(transformation.data());

		/*if(detectedPlanes > 0){
			ofxVec3f v1,v2,v3,v4;
			if(false)
				cout << "x2: " << detectedPlane.getVA().x << " y2: " << detectedPlane.getVA().y << " z2: " << detectedPlane.getVA().z << endl;
			v1 = POINTXYZ_OFXVEC3F(detectedPlane.cloudQuad.at(detectedPlane.getVertexIdxs()->indices.at(0)));
			v1 = kinect->getScreenCoordsFromWorldCoords(v1);
			v2 = POINTXYZ_OFXVEC3F(detectedPlane.cloudQuad.at(detectedPlane.getVertexIdxs()->indices.at(1)));
			v2 = kinect->getScreenCoordsFromWorldCoords(v2);
			v3 = POINTXYZ_OFXVEC3F(detectedPlane.cloudQuad.at(detectedPlane.getVertexIdxs()->indices.at(2)));
			v3 = kinect->getScreenCoordsFromWorldCoords(v3);
			v4 = POINTXYZ_OFXVEC3F(detectedPlane.cloudQuad.at(detectedPlane.getVertexIdxs()->indices.at(3)));
			v4 = kinect->getScreenCoordsFromWorldCoords(v4);

			ofSetColor(kRGBBlue);
			ofCircle(v1.x,v1.y, 5);
			ofSetColor(0,0,0);
			ofCircle(v2.x,v2.y, 5);
			ofSetColor(0,255,0);
			ofCircle(v3.x,v3.y, 5);
			ofSetColor(255,0,0);
			ofCircle(v4.x,v4.y, 5);
		}*/

		/*ofSetColor(kRGBRed);
		glBegin(GL_POINTS);
		for (int i = 0; i < vCloudHull.size(); i += 1) {
			glVertex3f(vCloudHull[i].x, vCloudHull[i].y, 0);
		}
		glEnd();*/
		for(int i = 0; i < MAX_OBJECTS; i++)
			if(tempObjectsStatus[i] > TIMES_TO_CREATE_OBJ)
				detectedObjects[i].draw();

		openCVService->contourFinder.draw(640,480,150,150);
		ofPopMatrix();
	}
}

//--------------------------------------------------------------
void PCM::drawPointCloud() {
	ofScale(400, 400, 400);
	int w = 640;
	int h = 480;

	ofRotateY(pointCloudRotationY);
	float* distancePixels = kinect->getDistancePixels();

	glBegin(GL_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			ofPoint cur = kinect->getWorldCoordinateFor(x, y);
			ofColor color = kinect->getCalibratedColorAt(x,y);
			glColor3ub((unsigned char)color.r,(unsigned char)color.g,(unsigned char)color.b);
			glVertex3f(cur.x, cur.y, cur.z);
		}
	}
	glEnd();

}

//--------------------------------------------------------------
void PCM::saveCloud(const string& name){
	cout << "saving: " << name << "..." << endl;

	register int centerX = (cloud->width >> 1);
	int centerY = (cloud->height >> 1);
	float bad_point = std::numeric_limits<float>::quiet_NaN();

	register float* depth_map = kinect->getDistancePixels();
	float constant = 0.5f; //CAlculado segun lo que hay en la libreria de PCL
	int step = 1;
	register int depth_idx = 0;
	for(int v = -centerY; v < centerY; ++v) {
		for(register int u = -centerX; u < centerX; ++u, ++depth_idx) {
			pcl::PointXYZ& pt = cloud->points[depth_idx];
			// Check for invalid measurements
			if (depth_map[depth_idx] == 0)
			{
				// not valid
				pt.x = pt.y = pt.z = bad_point;
				continue;
			}
			pt.z = depth_map[depth_idx];
			pt.x = u * constant;
			pt.y = v * constant;
			//pt.x = u * pt.z * constant;
			//pt.y = v * pt.z * constant;
		}
	}
	//pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	/*PointCloud<PointXYZ>::Ptr cloud_source_ptr; 

	cloud_source_ptr = cloud->makeShared(); */
	//viewer.showCloud (cloud);

	string filename = name + PCD_EXTENSION;
	pcl::io::savePCDFileASCII (filename, *cloud);
	std::cerr << "Saved " << cloud->points.size () << " data points to " << filename << std::endl;
}

void PCM::savePartialCloud(ofPoint min, ofPoint max, int id, const string& name){
	//Calcular tamaño de la nube
	cout << "saving: " << name << "..." << endl;
	PointCloud<PointXYZ>::Ptr partialColud = getPartialCloud(min,max);
	std::stringstream ss;
	ss << name << "-" << id << PCD_EXTENSION;
	pcl::io::savePCDFileASCII (ss.str(), *partialColud);
	std::cerr << "Saved " << partialColud->points.size () << " data points to " << ss.str() << std::endl;
}

PointCloud<PointXYZ>* PCM::loadCloud(const string& name) {
	cout << "loading: "<< name << "..."<<endl;
	pcl::PointCloud<pcl::PointXYZ>* tmpCloud = new pcl::PointCloud<PointXYZ>();
	string filename = name + PCD_EXTENSION;
	pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *tmpCloud);
	cout << name << " loaded!"<<endl;
	return tmpCloud;
}

PointCloud<PointXYZ>::Ptr PCM::getCloud(){
	return getPartialCloudRealCoords(ofPoint(0,0),ofPoint(KINECT_WIDTH,KINECT_HEIGHT));
}

PointCloud<PointXYZRGB>::Ptr PCM::getColorCloud(){
	return getPartialColorCloud(ofPoint(0,0),ofPoint(KINECT_WIDTH,KINECT_HEIGHT));
}


PointCloud<PointXYZ>::Ptr PCM::getPartialCloud(ofPoint min, ofPoint max){
	//Calcular tamaño de la nube
	PointCloud<PointXYZ>::Ptr partialColud (new pcl::PointCloud<pcl::PointXYZ>);
	partialColud->width    = max.x - min.x;
	partialColud->height   = max.y - min.y;
	partialColud->is_dense = false;
	partialColud->points.resize (partialColud->width*partialColud->height);

	//Recorrer el mapa de distancias obteniendo sólo los que estén dentro del rectángulo

	register int centerX = (cloud->width >> 1);
	int centerY = (cloud->height >> 1);
	float bad_point = std::numeric_limits<float>::quiet_NaN();

	register float* depth_map = kinect->getDistancePixels();

	int step = 1;
	register int depth_idx = 0;
	int absV, absU, cloud_idx = 0;
	for(int v = -centerY; v < centerY; ++v) {
		for(register int u = -centerX; u < centerX; ++u, ++depth_idx) {
			absV = v + centerY;
			absU = u + centerX;
			if( absV > min.y && absV < max.y && absU > min.x && absU < max.x)
			{
				pcl::PointXYZ& pt = partialColud->points[cloud_idx];

				// Check for invalid measurements o puntos que estan fuera del nearthreshold/farthreshold
				// TODO: limitar que depth_map[depth_idx] caiga dentro de nearThreshold/farThreshold
				// Lo intente hacer pero parece que esos valores no corresponden con lo que lee el kinect
				if(/*depth_map[depth_idx] > 100 || */depth_map[depth_idx] == 0){
					pt.x = pt.y = pt.z = 0;
					//continue;
				}
				else{
					//ofxVec3f point = kinect->getWorldCoordinateFor(u,v);
					
					pt.z = depth_map[depth_idx];
					pt.x = u;
					pt.y = v;

				}
				cloud_idx++;
			}
		}
	}
	//pcl::io::savePCDFileASCII ("test_pacial_pcd.pcd", *partialColud);
	return partialColud;	
}

PointCloud<PointXYZ>::Ptr PCM::getPartialCloudRealCoords(ofPoint min, ofPoint max, int density){
	//Chequeo
	if (min.x < 0 || min.y < 0 || max.x > KINECT_WIDTH || max.y > KINECT_HEIGHT || min.x > max.x || min.y > max.y) //* load the file
	{
		PCL_ERROR ("Error en parametros de entrada para obtener la nube \n");
	}
	
	//Calcular tamaño de la nube
	PointCloud<PointXYZ>::Ptr partialColud (new pcl::PointCloud<pcl::PointXYZ>);
	partialColud->width    = ceil((max.x - min.x)/density);
	partialColud->height   = ceil((max.y - min.y)/density);
	partialColud->is_dense = false;
	partialColud->points.resize (partialColud->width*partialColud->height);
	register float* depth_map = kinect->getDistancePixels();
	//Recorrer el mapa de distancias obteniendo sólo los que estén dentro del rectángulo
	register int depth_idx = 0;
	int cloud_idx = 0;
	for(int v = min.y; v < max.y; v += density) {
		for(register int u = min.x; u < max.x; u += density) {
			pcl::PointXYZ& pt = partialColud->points[cloud_idx];
			cloud_idx++;
			
			// Check for invalid measurements
			if(depth_map[depth_idx] == 0){
				pt.x = pt.y = pt.z = 0;
			}
			else{
				ofxVec3f pto = kinect->getWorldCoordinateFor(u,v);
				pt.x = pto.x;
				pt.y = pto.y;
				pt.z = pto.z;
			}

			depth_idx += density;
		}
	}
	//pcl::io::savePCDFileASCII ("partial_real.pcd", *partialColud);
	return partialColud;	
}

//No encaró
PointCloud<PointXYZRGB>::Ptr PCM::getPartialColorCloud(ofPoint min, ofPoint max){
	//Calcular tamaño de la nube
	PointCloud<PointXYZRGB>::Ptr partialColud (new pcl::PointCloud<pcl::PointXYZRGB>);
	partialColud->width    = max.x - min.x;
	partialColud->height   = max.y - min.y;
	partialColud->is_dense = false;
	partialColud->points.resize (partialColud->width*partialColud->height);

	//Recorrer el mapa de distancias obteniendo sólo los que estén dentro del rectángulo

	register int centerX = (cloud->width >> 1);
	int centerY = (cloud->height >> 1);
	float bad_point = std::numeric_limits<float>::quiet_NaN();

	register float* depth_map = kinect->getDistancePixels();
	unsigned char* color_map = kinect->getCalibratedRGBPixels();
	float constant = 0.5f; //CAlculado segun lo que hay en la libreria de PCL
	int step = 1;
	register int depth_idx = 0;
	int absV, absU, cloud_idx = 0;
	for(int v = -centerY; v < centerY; ++v) {
		for(register int u = -centerX; u < centerX; ++u, ++depth_idx) {
			absV = v + centerY;
			absU = u + centerX;
			if( absV > min.y && absV < max.y && absU > min.x && absU < max.x)
			{
				pcl::PointXYZRGB& pt = partialColud->points[cloud_idx];

				// Check for invalid measurements o puntos que estan fuera del nearthreshold/farthreshold
				// TODO: limitar que depth_map[depth_idx] caiga dentro de nearThreshold/farThreshold
				// Lo intente hacer pero parece que esos valores no corresponden con lo que lee el kinect
				if(/*depth_map[depth_idx] > 100 || */depth_map[depth_idx] == 0){
					pt.x = pt.y = pt.z = bad_point;
					//continue;
				}
				else{
					pt.z = depth_map[depth_idx];
					pt.x = u * constant;
					pt.y = v * constant;
					ofColor c = kinect->getColorAt(absU,absV);
					//pt.r = c.r*255;
					//pt.g = c.g*255;
					//pt.b = c.b*255;
				}


				cloud_idx++;
			}
		}
	}
	//pcl::io::savePCDFileASCII ("test_pacial_pcd.pcd", *partialColud);
	return partialColud;	
}

//--------------------------------------------------------------
void PCM::setInitialPointCloud(){

	cloud = getCloud();
	currentDiffcloud = cloud;
	//saveCloud(DEFAULT_NAME);
	//cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(loadCloud(DEFAULT_NAME));

	//pcl::io::savePCDFileASCII ("test_inicial.pcd", *cloud);
	////// assign point cloud to octree
	//   octree->setInputCloud(capturedCloud);

	//   // add points from cloud to octree
	//   octree->addPointsFromInputCloud();

	//octree->switchBuffers();
}

//--------------------------------------------------------------
PointCloud<PointXYZ>::Ptr PCM::getDifferenceIdx(bool &isDif, int noise_filter){
	PointCloud<PointXYZ>::Ptr difCloud (new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr secondCloud =  getCloud();
	isDif = false;

	/*pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("base.pcd", *currentDiffcloud, false);
	writer.write<pcl::PointXYZ> ("dif.pcd", *secondCloud, false);
*/

	int dif = getDifferencesCloud(currentDiffcloud,secondCloud,difCloud,OCTREE_RES);
	if(dif > DIFF_THRESHOLD || noDifferencesCount < 25){
		//cout << dif << endl;
		if(dif < DIFF_THRESHOLD)
			noDifferencesCount++;
		else
			noDifferencesCount = 0;

		currentDiffcloud = secondCloud;
		dif = getDifferencesCloud(cloud,secondCloud,difCloud,OCTREE_RES);
		if(dif  > MIN_DIFF_TO_PROCESS){
			isDif = true;
		}
	}

	return difCloud;
}

//--------------------------------------------------------------
void PCM::processDiferencesClouds(){
	bool dif;
	//nDetectedObjects = openCVService->contourFinder.blobs.size();
	PointCloud<PointXYZ>::Ptr filteredCloud = getDifferenceIdx(dif);
	if(dif){
		
		//Subdivido la nube de diferencias en clusters
		// Creating the KdTree object for the search method of the extraction
		pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
		tree->setInputCloud (filteredCloud);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance (0.02); // 2cm
		ec.setMinClusterSize (100);
		ec.setMaxClusterSize (25000);
		ec.setSearchMethod (tree);
		ec.setInputCloud(filteredCloud);
		ec.extract (cluster_indices);

		/*nDetectedObjects = cluster_indices.size();
		int j = 0;
*/

		//Actualizo las detecciones temporales
		for(int i = 0; i < MAX_OBJECTS; i++)
			if(tempObjectsStatus[i] > 0)
				tempObjectsStatus[i]--;

		//separo en clusters y valido si existen nuevos objetos o actualizo ya detectados
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
				cloud_cluster->points.push_back (filteredCloud->points[*pit]); //*

			if(!updateDetectedObject(cloud_cluster))
			{
				//detectedObjects[nDetectedObjects] = Object3D(cloud_cluster,cloud_cluster,kinect);
				int slot = getSlotForTempObj();
				if(slot > -1)
				{
					tempObjects[slot] = *cloud_cluster;
					tempObjectsStatus[slot] = 2;
				}
				//nDetectedObjects++;
			}
			/*detectedObjects[j] = Object3D(cloud_cluster,cloud_cluster,kinect);
			j++;*/
		}
	}
}

int PCM::getSlotForTempObj(){
	for(int i = 0; i < MAX_OBJECTS; i++)
			if(tempObjectsStatus[i] == 0)
				return i;
	return -1;
}

bool PCM::updateDetectedObject(PointCloud<PointXYZ>::Ptr cloud_cluster){
	PointCloud<PointXYZ>::Ptr difCloud (new PointCloud<PointXYZ>);
	for (int i = 0; i < MAX_OBJECTS; i ++){
		if(tempObjectsStatus[i] > 0)
		{
			PointCloud<PointXYZ>::Ptr objCloud (new PointCloud<PointXYZ> (tempObjects[i])); 
			int dif = getDifferencesCloud(objCloud,cloud_cluster,difCloud,RES_IN_OBJ);
			if(dif < DIFF_IN_OBJ)
			{
				if(tempObjectsStatus[i] < TIMES_TO_CREATE_OBJ)
					tempObjectsStatus[i]+=2;
				else
					tempObjectsStatus[i]++;

				if(tempObjectsStatus[i] == TIMES_TO_CREATE_OBJ)
				{
					tempObjectsStatus[i] = TIMES_TO_CREATE_OBJ*2;
					detectedObjects[i] = Object3D(cloud_cluster,cloud_cluster,kinect);
					nDetectedObjects++;
				}
				return true;
			}
		}
	}

	return false;
}
//--------------------------------------------------------------
//void PCM::detectPlanes(PointCloud<PointXYZ>::Ptr currentCloud){
//	sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2());
//	sensor_msgs::PointCloud2::Ptr cloud_filtered_blob (new sensor_msgs::PointCloud2);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
//
//
//	//Remover outliers
//	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//	sor.setInputCloud (currentCloud);
//	sor.setMeanK (10); //Cantidad de vecinos a analizar
//	sor.setStddevMulThresh (1.0);
//	sor.filter (*cloud_filtered);
//
//
//	///Comentada la parte de downsampling porque no habia gran diferencia
//	//pcl::toROSMsg<pcl::PointXYZ>(*currentCloud, *cloud_blob);
//
//	//std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
//
//	//// Create the filtering object: downsample the dataset using a leaf size of 1cm
//	//pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
//	//sor.setInputCloud (cloud_blob);
//	//sor.setLeafSize (0.01, 0.01, 0.01);
//	//sor.filter (*cloud_filtered_blob);
//
//	//// Convert to the templated PointCloud
//	//pcl::fromROSMsg (*cloud_filtered_blob, *cloud_filtered);
//
//	//std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
//	///
//
//	////Esto es si esta comentado lo de downsampling
//	//cloud_filtered = currentCloud;
//	///////////
//
//	// Write the downsampled version to disk
//	pcl::PCDWriter writer;
//	/*writer.write<pcl::PointXYZ> ("box_downsampled.pcd", *cloud_filtered, false);*/
//
//	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
//	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
//	// Create the segmentation object
//	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	// Optional
//	seg.setOptimizeCoefficients (true);
//	// Mandatory
//	seg.setModelType (pcl::SACMODEL_PLANE);
//	seg.setMethodType (pcl::SAC_RANSAC);
//	seg.setMaxIterations (50);
//	seg.setDistanceThreshold (0.009); //original: 0.01
//
//	// Create the filtering object
//	pcl::ExtractIndices<pcl::PointXYZ> extract;
//
//	int i = 0, nr_points = cloud_filtered->points.size ();
//	// mientras 10% de la nube no se haya procesado
//	detectedPlanesNumber = 0;
//	while (cloud_filtered->points.size () > 0.1 * nr_points && detectedPlanesNumber < MAX_PLANES)
//	{
//		// Segment the largest planar component from the remaining cloud
//		seg.setInputCloud (cloud_filtered);
//		seg.segment (*inliers, *coefficients);
//		if (inliers->indices.size () == 0)
//		{
//			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
//			break;
//		}
//
//		// Extract the inliers
//		extract.setInputCloud (cloud_filtered);
//		extract.setIndices (inliers);
//		extract.setNegative (false);
//		extract.filter (*cloud_p);
//		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
//		// Create the filtering object
//		extract.setInputCloud (cloud_filtered);
//		extract.setIndices (inliers);
//		extract.setNegative (true);
//
//		//FIX
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_temp (new pcl::PointCloud<pcl::PointXYZ>());
//		if(cloud_p->size() != cloud_filtered->size())
//			extract.filter (*cloud_filtered_temp);
//		
//		cloud_filtered = cloud_filtered_temp;
//		
//		// Create a Convex Hull representation of the projected inliers
//		//Comento convexHull para ver si mejora los tiempos
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
//		pcl::ConvexHull<pcl::PointXYZ> chull;
//		chull.setInputCloud (cloud_p);
//		chull.reconstruct (*cloud_hull);
//
//		/**/
//		/*std::stringstream ss;
//		ss << "box_plane_" << i << ".pcd";
//		writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);*/
//		//Salvo a memoria en lugar de escribir en archivo
//		planes[detectedPlanesNumber] = PointCloud<pcl::PointXYZ>(*cloud_p);
//		detectedPlanes++;
//
//		if (cloud_hull->size() == 0) {
//			continue;
//		}
//
//		//Comento para que no grabe a disco
//		//std::stringstream ss2;
//		//ss2 << "box_plane_hull_" << i << PCD_EXTENSION;
//		//writer.write<pcl::PointXYZ> (ss2.str (), *cloud_hull, false);
//
//		
//		time_t now = time(NULL);
//
//		vCloudHull.clear();
//
//		for (int k = 0; k < cloud_p->size(); k++) {
//			vCloudHull.push_back(POINTXYZ_OFXVEC3F(cloud_p->at(k)));
//		}
//
//		detectedPlane.findQuad(vCloudHull);
//		//detectedPlane.avgNormal = normalEstimation(cloud_p);
//		detectedPlane.cloudQuad = *cloud_p;
//		ofxVec3f vMin = kinect->getScreenCoordsFromWorldCoords(detectedPlane.vMin);
//		ofxVec3f vMax = kinect->getScreenCoordsFromWorldCoords(detectedPlane.vMax);
//
//		detectedPlane.extendedcloudQuad = getPartialCloudRealCoords(ofPoint(vMin.x - QUAD_HALO,vMin.y - QUAD_HALO),
//												  ofPoint(vMax.x + QUAD_HALO ,vMax.y + QUAD_HALO));
//
//		//detectedPlane.vertexNormal = normalEstimation(cloud_p, detectedPlane.getVertexIdxs());
//		
//		cout << "x2: " << detectedPlane.getVA().x << " y2: " << detectedPlane.getVA().y << " z2: " << detectedPlane.getVA().z << endl;
//		////DEBUG
//		//pcl::PointCloud<pcl::PointXYZ>::Ptr vertCloud (new pcl::PointCloud<pcl::PointXYZ>());
//		//vertCloud->resize(4);
//		//for(int i = 0; i < 4; i++)
//		//	vertCloud->points[i] = detectedPlane.cloudQuad->points[detectedPlane.getVertexIdxs()->indices.at(i)];
//		//writer.write<pcl::PointXYZ> ("debug.pcd", *detectedPlane.cloudQuad, false);
//		//writer.write<pcl::PointXYZ> ("debug_vertex.pcd", *vertCloud, false);
//
//
//		cout << time(NULL) - now << endl;
//
//		i++;
//	}
//
//	cout << "Detected planes: " << detectedPlanes << endl;
//
//
//}

//--------------------------------------------------------------
ofxVec3f PCM::normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr plane){
	//Calculo de normales
		//Random indices
	int sampleSize = floor (plane->points.size() / 10.0f);
	std::vector<int> indices (sampleSize);
	for (size_t i = 0; i < sampleSize; i++) 
		indices[i] = rand() % plane->points.size();

	pcl::PointIndices::Ptr indicesptr (new pcl::PointIndices ());
	indicesptr->indices = indices;


	return normalEstimation(plane, indicesptr);
}

//--------------------------------------------------------------
ofxVec3f PCM::normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr plane, pcl::PointIndices::Ptr indicesptr){
	//Calculo de normales
	int sampleSize = indicesptr->indices.size();
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (plane);
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	
	ne.setIndices(indicesptr);
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.3);

	// Compute the features
	ne.compute (*cloud_normals);

	ofxVec3f result(0,0,0);
	//Promedio las normales
	float bad_point = std::numeric_limits<float>::quiet_NaN();
	for(int i = 0; i < sampleSize ; i++){
		pcl::Normal normal = cloud_normals->at(i);
		if (normal.normal_z != bad_point && normal.normal_z == normal.normal_z)
			result += PCLNORMAL_OFXVEC3F(cloud_normals->at(i));
	}
	
	result /= sampleSize;
	result = result.normalize();
	return result;
}

//--------------------------------------------------------------
void PCM::setTransformation(){
	//pcl::PCDWriter writer;
	//ofxVec3f vMin = kinect->getScreenCoordsFromWorldCoords(detectedPlane.vMin);
	//ofxVec3f vMax = kinect->getScreenCoordsFromWorldCoords(detectedPlane.vMax);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud = getPartialCloudRealCoords(ofPoint(vMin.x - QUAD_HALO,vMin.y - QUAD_HALO),
	//											ofPoint(vMax.x + QUAD_HALO,vMax.y + QUAD_HALO));
	//pcl::PointCloud<pcl::PointXYZ> target_transformed;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr src_filtered  (new PointCloud<PointXYZ> ());
	//pcl::PointCloud<pcl::PointXYZ>::Ptr target_filtered (new PointCloud<PointXYZ> ());
	//float closestTarget,closestSrc;

	////Filtrado 
	//closestTarget = getNearestPoint(newCloud);
	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setFilterFieldName ("z");

	//pass.setFilterLimits (closestTarget, closestTarget + .4);
	//pass.setInputCloud (newCloud);
	//pass.filter (*target_filtered);

	//closestSrc = getNearestPoint(detectedPlane.extendedcloudQuad);
	//pass.setFilterLimits (closestSrc, closestSrc + .4);
	//pass.setInputCloud (detectedPlane.extendedcloudQuad);
	//pass.filter (*src_filtered);


	//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//icp.setMaximumIterations(100);
	//icp.setMaxCorrespondenceDistance(0.004);
	//icp.setTransformationEpsilon (1e-10);

	//icp.setInputCloud(src_filtered);
	//icp.setInputTarget(target_filtered);
	//icp.align(target_transformed);

	//transformation *= icp.getFinalTransformation();

	//writer.write<pcl::PointXYZ> ("cludsource.pcd", detectedPlane.cloudQuad, false);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::transformPointCloud(detectedPlane.cloudQuad, *temp,transformation);
	//detectedPlane.cloudQuad = *temp;
	//
	//writer.write<pcl::PointXYZ> ("cludsourcetemp.pcd", *temp, false);
	//
	////transEstimation.estimateRigidTransformation(*detectedPlane.extendedcloudQuad,indices,*newCloud,transformation);
	///*cout << transformation << endl*/
	//writer.write<pcl::PointXYZ> ("source.pcd", *src_filtered, false);
	//writer.write<pcl::PointXYZ> ("target.pcd", *target_filtered, false);
	//writer.write<pcl::PointXYZ> ("target_trans.pcd", target_transformed, false);
}

//--------------------------------------------------------------
void PCM::keyPressed (int key) {
	switch (key) {
	case ' ':
		detectMode = !detectMode;
		break;
	case'p':
		drawPC = !drawPC;
		break;
	case 's':
		saveCloud(DEFAULT_NAME);
		break;
	case 'i':
		//setInitialPointCloud();
		icp();
		break;
	case 'd':
		processDiferencesClouds();
		break;
	case 't':
		setTransformation();
		break;
	}
}

//--------------------------------------------------------------
void PCM::mouseMoved(int x, int y)
{
	pointCloudRotationY = x;
}

//--------------------------------------------------------------
void PCM::mouseDragged(int x, int y, int button)
{

}

//--------------------------------------------------------------
void PCM::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void PCM::windowResized(int w, int h)
{

}

//--------------------------------------------------------------
void PCM::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void PCM::icp(){

	ofxVec3f w = kinect->getWorldCoordinateFor(20,20);
	ofxVec3f s = kinect->getScreenCoordsFromWorldCoords(w);

	int iterator = 0;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr src (new PointCloud<PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target (new PointCloud<PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> target_transformed, target_acumulative_transformed, final;
	pcl::PointCloud<pcl::PointXYZ> src_filtered;
	pcl::PointCloud<pcl::PointXYZ> target_filtered;
	Eigen::Matrix4f totalTransformation;
	totalTransformation.setIdentity();
	//Obtengo la nube base
	src = getPartialCloudRealCoords(ofPoint(0,0),ofPoint(KINECT_WIDTH,KINECT_HEIGHT),6);

	//filtrado
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (.40, .75);

	pass.setInputCloud (src);
	pass.filter (src_filtered);

	while(true){
		std::cerr<<"Cambio!"<< endl;
		getchar();
		kinect->update();

		//Obtengo la segunda nube
		target = getPartialCloudRealCoords(ofPoint(0,0),ofPoint(KINECT_WIDTH,KINECT_HEIGHT),6);
	
		//filtrado
		pass.setInputCloud (target);
		pass.filter (target_filtered);
	
		//Seteo propiedades ICP
		
		icp.setMaximumIterations(20); // Cantidad maxima de iteraciones
		
		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance(0.005); // Distancia maxima en la que se toma la correspondencia de los puntos (en metros)
		
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon (1e-8); // Diferencia mínima entre una transformacion y la siguiente iteracion
		
		
		cout << "Source: " << src_filtered.size() << endl;
		cout << "Target: " << target_filtered.size() << endl;
		pcl::io::savePCDFileASCII ("icp_src" + ofToString(iterator) + ".pcd", src_filtered);
		pcl::io::savePCDFileASCII ("icp_target" + ofToString(iterator) + ".pcd", target_filtered);

		
		time_t now = time(NULL);

		PointCloud<PointXYZ>::Ptr src_filteredPTR (new PointCloud<PointXYZ> (src_filtered));
		PointCloud<PointXYZ>::Ptr target_filteredPTR (new PointCloud<PointXYZ> (target_filtered));

		icp.setInputCloud(target_filteredPTR);
		icp.setInputTarget(src_filteredPTR);
		icp.align(target_transformed);


		cout << time(NULL) - now << endl;


		std::cout << "has converged:" << icp.hasConverged() << " score: " 
				  << icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;

		
		
		//pcl::io::savePCDFileASCII ("icp_src_transformed" + ofToString(iterator) + ".pcd", *src_transformed);

		//Downsampling
		/*pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
		sor.setInputCloud (*second_filtered);
		sor.setLeafSize (0.01, 0.01, 0.01);
		sor.filter (*Final);*/

		//Remove Outliers
		/*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud (src_transformed);
		sor.setMeanK (40);
		sor.setStddevMulThresh (5.0);
		sor.filter (*src_filtered);*/
		if(iterator == 0){
			final += src_filtered;
			target_acumulative_transformed = target_transformed;
			totalTransformation *= icp.getFinalTransformation();
		}
		else
		{
			pcl::transformPointCloud(target_transformed,target_acumulative_transformed,totalTransformation);
			totalTransformation *= icp.getFinalTransformation();
		}

		final += target_acumulative_transformed;
		


		src_filtered = target_filtered;

		pcl::io::savePCDFileASCII ("icp_target_acu" + ofToString(iterator) + ".pcd", target_acumulative_transformed);
		pcl::io::savePCDFileASCII ("icp_target_transformed" + ofToString(iterator) + ".pcd", target_transformed);
		pcl::io::savePCDFileASCII ("icp" + ofToString(iterator) + ".pcd", final);
		iterator++;
	}
}
