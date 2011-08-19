#include "PCM.h"

#include "pointUtils.h"
#include "ofxVecUtils.h"
#include "Line2D.h"
#include "Triangle2D.h"
#include "ofGraphicsUtils.h"

#define DEFAULT_NAME		"test"

using namespace std;

//--------------------------------------------------------------
void PCM::setup(ofxKinect *kinect) {
	this->kinect = kinect;

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
		else if(ofGetElapsedTimef() - timer > 5) {
			cout << "Process diferences" << endl;
			//processDiferencesClouds();
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
		kinect->drawDepth(10, 10, 400, 300);
		ofResetColor();
		ofPushMatrix();
		int w_2 = 640 / 2;
		int h_2 = 480 / 2;
		ofTranslate(420 + w_2, 10 + h_2);
		kinect->draw(-w_2, -h_2);

		ofEnableAlphaBlending();
		ofSetColor(255,255,255,128);
		kinect->drawDepth(-w_2, -h_2);
		ofDisableAlphaBlending();

		ofSetColor(kRGBGreen);
		ofTriangle(detectedPlane.getVA().x, detectedPlane.getVA().y,
			detectedPlane.getVB().x, detectedPlane.getVB().y,
			detectedPlane.getVC().x, detectedPlane.getVC().y);
		ofTriangle(detectedPlane.getVA().x, detectedPlane.getVA().y,
			detectedPlane.getVB().x, detectedPlane.getVB().y,
			detectedPlane.getVD().x, detectedPlane.getVD().y);

		ofSetColor(kRGBBlue);
		ofCircle(detectedPlane.getVA().x, detectedPlane.getVA().y, 5);
		ofSetColor(0,0,192);
		ofCircle(detectedPlane.getVB().x, detectedPlane.getVB().y, 5);
		ofSetColor(0,0,128);
		ofCircle(detectedPlane.getVC().x, detectedPlane.getVC().y, 5);
		ofSetColor(0,0,64);
		ofCircle(detectedPlane.getVD().x, detectedPlane.getVD().y, 5);

		ofSetColor(kRGBRed);
		glBegin(GL_POINTS);
		for (int i = 0; i < vCloudHull.size(); i += 1) {
			glVertex3f(vCloudHull[i].x, vCloudHull[i].y, 0);
		}
		glEnd();

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
	return getPartialCloud(ofPoint(0,0),ofPoint(KINECT_WIDTH,KINECT_HEIGHT));
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
					pt.x = pt.y = pt.z = bad_point;
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
PointCloud<PointXYZ>::Ptr PCM::getDifferenceIdx(bool &dif, int noise_filter){
	// Instantiate octree-based point cloud change detection class
	octree::OctreePointCloudChangeDetector<PointXYZ> octree (OCTREE_RES);
	std::vector<int> newPointIdxVector;

	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	octree.switchBuffers();

	PointCloud<PointXYZ>::Ptr secondCloud =  getCloud();

	//pcl::io::savePCDFileASCII ("initial.pcd", *cloud);
	//pcl::io::savePCDFileASCII ("second.pcd", *secondCloud);

	octree.setInputCloud(secondCloud);
	octree.addPointsFromInputCloud();

	octree.getPointIndicesFromNewVoxels (newPointIdxVector);

	std::cerr << newPointIdxVector.size() << std::endl;
	PointCloud<PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);

	if(newPointIdxVector.size() > MIN_DIFF_TO_PROCESS)
	{
		filteredCloud->points.reserve(newPointIdxVector.size());

		for (std::vector<int>::iterator it = newPointIdxVector.begin(); it != newPointIdxVector.end(); it++)
			filteredCloud->points.push_back(secondCloud->points[*it]);

		std::stringstream ss;
		ss << "difference" << PCD_EXTENSION;
		//pcl::io::savePCDFileASCII (ss.str(), *filteredCloud);

		dif = true;
		//myoctree->switchBuffers();
	}
	else
		dif = false;

	return filteredCloud;
}

//--------------------------------------------------------------
void PCM::processDiferencesClouds(){
	bool dif;
	PointCloud<PointXYZ>::Ptr filteredCloud = getDifferenceIdx(dif);
	if(dif)
		detectPlanes(filteredCloud);
	else
		cout << "No differences founded.";
}

//--------------------------------------------------------------
void PCM::detectPlanes(PointCloud<PointXYZ>::Ptr currentCloud){
	sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2());
	sensor_msgs::PointCloud2::Ptr cloud_filtered_blob (new sensor_msgs::PointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>);



	///Comentada la parte de downsampling porque no habia gran diferencia
	//pcl::toROSMsg<pcl::PointXYZ>(*currentCloud, *cloud_blob);

	//std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

	//// Create the filtering object: downsample the dataset using a leaf size of 1cm
	//pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	//sor.setInputCloud (cloud_blob);
	//sor.setLeafSize (0.01, 0.01, 0.01);
	//sor.filter (*cloud_filtered_blob);

	//// Convert to the templated PointCloud
	//pcl::fromROSMsg (*cloud_filtered_blob, *cloud_filtered);

	//std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
	///

	////Esto es si esta comentado lo de downsampling
	cloud_filtered = currentCloud;
	///////////

	// Write the downsampled version to disk
	pcl::PCDWriter writer;
	/*writer.write<pcl::PointXYZ> ("box_downsampled.pcd", *cloud_filtered, false);*/

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (50);
	seg.setDistanceThreshold (0.9); //original: 0.01

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	int i = 0, nr_points = cloud_filtered->points.size ();
	// mientras 10% de la nube no se haya procesado
	detectedPlanes = 0;
	while (cloud_filtered->points.size () > 0.1 * nr_points && detectedPlanes < MAX_PLANES)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_p);
		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;


		// Create a Convex Hull representation of the projected inliers
		//Comento convexHull para ver si mejora los tiempos
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ConvexHull<pcl::PointXYZ> chull;
		chull.setInputCloud (cloud_p);
		chull.reconstruct (*cloud_hull);

		/**/
		/*std::stringstream ss;
		ss << "box_plane_" << i << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);*/
		//Salvo a memoria en lugar de escribir en archivo
		planes[detectedPlanes] = PointCloud<pcl::PointXYZ>(*cloud_p);
		detectedPlanes++;

		if (cloud_hull->size() == 0) {
			continue;
		}

		//Comento para que no grabe a disco
		//std::stringstream ss2;
		//ss2 << "box_plane_hull_" << i << PCD_EXTENSION;
		//writer.write<pcl::PointXYZ> (ss2.str (), *cloud_hull, false);

		// Create the filtering object
		extract.setNegative (true);

		//FIX
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_temp (new pcl::PointCloud<pcl::PointXYZ>(*cloud_filtered));
		extract.filter (*cloud_filtered_temp);
		cloud_filtered = cloud_filtered_temp;

		time_t now = time(NULL);

		vCloudHull.clear();

		for (int k = 0; k < cloud_p->size(); k++) {
			vCloudHull.push_back(POINTXYZ_OFXVEC3F(cloud_p->at(k)));
		}

		detectedPlane.findQuad(vCloudHull);

		cout << time(NULL) - now << endl;

		i++;
	}

	cout << "Detected planes: " << detectedPlanes << endl;


}

//--------------------------------------------------------------
void PCM::keyPressed (int key) {
	switch (key) {
	case ' ':
		processDiferencesClouds();
		break;
	case'p':
		drawPC = !drawPC;
		break;
	case 's':
		saveCloud(DEFAULT_NAME);
		break;
	case 'i':
		setInitialPointCloud();
		break;
	case 'd':
		processDiferencesClouds();
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
	kinect->setCameraTiltAngle(0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr initial_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr initial = getCloud();
	std::stringstream ss;
	//ss << "icp_initial.pcd";
	//pcl::io::savePCDFileASCII (ss.str(), *initial);

	std::cerr<<"Cambio!"<< endl;
	//getchar();
	kinect->setCameraTiltAngle(0);
	ofSleepMillis(1000);
	this->update(kinect->isFrameNew());

	pcl::PointCloud<pcl::PointXYZ>::Ptr second_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr second = getCloud();
	//ss << "icp_second.pcd";
	//pcl::io::savePCDFileASCII (ss.str(), *second);

	//filtrado
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (initial);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (1.0, 1000.0);
	pass.filter (*initial_filtered);

	pass.setInputCloud (second);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (1.0, 1000.0);
	pass.filter (*second_filtered);

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	cout << "Initial: " << initial_filtered.get()->size() << endl;
	cout << "Second: " << second_filtered.get()->size() << endl;

	icp.setMaximumIterations(50);
	//icp.setMaxCorrespondenceDistance(4);
	icp.setInputCloud(initial_filtered);
	icp.setInputTarget(second_filtered);

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	ss << "icp.pcd";
	pcl::io::savePCDFileASCII (ss.str(), Final);
	kinect->setCameraTiltAngle(0);
}
