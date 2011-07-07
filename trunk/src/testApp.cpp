#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
	//kinect.init(true);  //shows infrared image
	kinect.init();
	kinect.setVerbose(true);
	kinect.open();

	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThresh.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	grayBg.allocate(kinect.width, kinect.height);
	grayDiff.allocate(kinect.width, kinect.height);

	nearThreshold = 233;
	farThreshold  = 208;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	pointCloudRotationY = 180;
	
	drawPC = false;
	bLearnBakground = true;
	threshold = 80;

	// Initialize point cloud
	cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
                (new pcl::PointCloud<pcl::PointXYZ>()); 
	cloud->width    = KINECT_WIDTH;
	cloud->height   = KINECT_HEIGHT;
	cloud->is_dense = false;
	cloud->points.resize (CLOUD_POINTS);

	// Inicializo octree
	myoctree = new octree::OctreePointCloudChangeDetector<PointXYZ>(0.01); //Valor de resolucion sacado del ejemplo
}

//--------------------------------------------------------------
void testApp::update() {
	ofBackground(100, 100, 100);
	
	kinect.update();
	if(kinect.isFrameNew())	// there is a new frame and we are connected
	{

		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
			
		//we do two thresholds - one for the far plane and one for the near plane
		//we then do a cvAnd to get the pixels which are a union of the two thresholds.	
		if( bThreshWithOpenCV ){
			grayThreshFar = grayImage;
			grayThresh = grayImage;
			grayThresh.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThresh.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		}else{
		
			//or we do it ourselves - show people how they can work with the pixels
		
			unsigned char * pix = grayImage.getPixels();
			int numPixels = grayImage.getWidth() * grayImage.getHeight();

			for(int i = 0; i < numPixels; i++){
				if( pix[i] < nearThreshold && pix[i] > farThreshold ){
					pix[i] = 255;
				}else{
					pix[i] = 0;
				}
			}
		}

		//if (bLearnBakground == true){
		//	grayBg = grayImage;		// the = sign copys the pixels from grayImage into grayBg (operator overloading)
		//	bLearnBakground = false;
		//}
		//// take the abs value of the difference between background and incoming and then threshold:
		//grayDiff.absDiff(grayBg, grayImage);
		//grayDiff.threshold(threshold);

		//update the cv image
		grayImage.flagImageChanged();
	
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
    	// also, find holes is set to true so we will get interior contours as well....
    	contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);

		//for(int i = 0 ; i < contourFinder.nBlobs; i++)
		//{
		//	cout << "min: (" << contourFinder.blobs[i].boundingRect.x << ", " << contourFinder.blobs[i].boundingRect.x + contourFinder.blobs[i].boundingRect.y << ")" << endl;
		//	cout << "max: (" << contourFinder.blobs[i].boundingRect.x + contourFinder.blobs[i].boundingRect.width << ", " << contourFinder.blobs[i].boundingRect.x + contourFinder.blobs[i].boundingRect.y + contourFinder.blobs[i].boundingRect.height<< ")" << endl;
		//}
	}
}

//--------------------------------------------------------------
void testApp::draw() {
	ofSetColor(255, 255, 255);
	if(drawPC){
		ofPushMatrix();
		ofTranslate(420, 320);
		// we need a proper camera class
		drawPointCloud();
		ofPopMatrix();
	}else{
		kinect.drawDepth(10, 10, 400, 300);
		kinect.draw(420, 10, 400, 300);

		grayImage.draw(10, 320, 400, 300);
		//grayDiff.draw(420, 10, 400, 300);
		contourFinder.draw(10, 320, 400, 300);
	}
	

	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
								 << ofToString(kinect.getMksAccel().y, 2) << " / " 
								 << ofToString(kinect.getMksAccel().z, 2) << endl
				 << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
				 << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
				 << "set near threshold " << nearThreshold << " (press: + -)" << endl
				 << "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
				 	<< ", fps: " << ofGetFrameRate() << endl
				 << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
				 << "press UP and DOWN to change the tilt angle: " << angle << " degrees";
	ofDrawBitmapString(reportStream.str(),20,666);
}

void testApp::drawPointCloud() {
	ofScale(400, 400, 400);
	int w = 640;
	int h = 480;
	ofRotateY(pointCloudRotationY);
	float* distancePixels = kinect.getDistancePixels();
	glBegin(GL_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			ofPoint cur = kinect.getWorldCoordinateFor(x, y);
			ofColor color = kinect.getCalibratedColorAt(x,y);
			glColor3ub((unsigned char)color.r,(unsigned char)color.g,(unsigned char)color.b);
			glVertex3f(cur.x, cur.y, cur.z);
		}
	}
	glEnd();
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void testApp::saveCloud(){
	register int centerX = (cloud->width >> 1);
	int centerY = (cloud->height >> 1);
	float bad_point = std::numeric_limits<float>::quiet_NaN();

	register float* depth_map = kinect.getDistancePixels();
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

	pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
	std::cerr << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;
}

void testApp::savePartialCloud(ofPoint min, ofPoint max, int id){
	//Calcular tamaño de la nube
	PointCloud<PointXYZ>::Ptr partialColud = getPartialCloud(min,max);
	std::stringstream ss;
	ss << "test_pcd-" << id << ".pcd";
	pcl::io::savePCDFileASCII (ss.str(), *partialColud);
	std::cerr << "Saved " << partialColud->points.size () << " data points to test_pcd.pcd." << std::endl;
}

PointCloud<PointXYZ>::Ptr testApp::getCloud(){
	return getPartialCloud(ofPoint(0,0),ofPoint(KINECT_WIDTH,KINECT_HEIGHT));
}

PointCloud<PointXYZ>::Ptr testApp::getPartialCloud(ofPoint min, ofPoint max){
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

	register float* depth_map = kinect.getDistancePixels();
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
				pcl::PointXYZ& pt = partialColud->points[cloud_idx];

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
				}


				cloud_idx++;
			}
		}
	}
	//pcl::io::savePCDFileASCII ("test_pacial_pcd.pcd", *partialColud);
	return partialColud;	
}

void testApp::captureBlobsClouds(){
	for (int i = 0; i < contourFinder.nBlobs; i ++){
		ofxCvBlob blob = contourFinder.blobs[i];
		ofPoint min (blob.boundingRect.x,blob.boundingRect.y);
		ofPoint max (blob.boundingRect.x + blob.boundingRect.width ,blob.boundingRect.y + blob.boundingRect.height);

		savePartialCloud(min,max,i);
	}
}

void testApp::processBlobsClouds(){
	for (int i = 0; i < contourFinder.nBlobs; i ++){
		ofxCvBlob blob = contourFinder.blobs[i];
		ofPoint min (blob.boundingRect.x,blob.boundingRect.y);
		ofPoint max (blob.boundingRect.x + blob.boundingRect.width ,blob.boundingRect.y + blob.boundingRect.height);

		PointCloud<PointXYZ>::Ptr currentCloud = getPartialCloud(min,max);
		detectPlanes(currentCloud);
	}
}

void testApp::setInitialPointCloud(){
	
	PointCloud<PointXYZ>::Ptr capturedCloud = getCloud();

	//// assign point cloud to octree
 //   octree.setInputCloud(capturedCloud);

 //   // add points from cloud to octree
 //   octree.addPointsFromInputCloud();

	//octree.switchBuffers();
}

PointCloud<PointXYZ>::Ptr testApp::getDifferenceIdx(const PointCloud<PointXYZ>::Ptr &cloud, int noise_filter){
	
	// Octree resolution - side length of octree voxels
	double resolution = 64;

	// Instantiate octree-based point cloud change detection class
	octree::OctreePointCloudChangeDetector<PointXYZ> octree (resolution);
	std::vector<int> newPointIdxVector;

	octree.setInputCloud(getCloud());
	octree.addPointsFromInputCloud();
	
	octree.switchBuffers();

	std::cerr<<"Cambio!"<< endl;
	getchar();
	this->update();

	octree.setInputCloud(getCloud());
	octree.addPointsFromInputCloud();
	
	octree.getPointIndicesFromNewVoxels (newPointIdxVector);
	
	std::cerr << newPointIdxVector.size() << std::endl;
	PointCloud<PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);


	filteredCloud->points.reserve(newPointIdxVector.size());

	for (std::vector<int>::iterator it = newPointIdxVector.begin(); it != newPointIdxVector.end(); it++)
		filteredCloud->points.push_back(cloud->points[*it]);

	std::stringstream ss;
	ss << "difference.pcd";
	pcl::io::savePCDFileASCII (ss.str(), *filteredCloud);

	//myoctree->switchBuffers();

	return filteredCloud;
}

void testApp::processDiferencesClouds(){
	PointCloud<PointXYZ>::Ptr currentCloud = getCloud();
	PointCloud<PointXYZ>::Ptr filteredCloud = getDifferenceIdx(currentCloud);

	detectPlanes(filteredCloud);
}
//--------------------------------------------------------------
void testApp::detectPlanes(PointCloud<PointXYZ>::Ptr currentCloud){
	sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2());
	sensor_msgs::PointCloud2::Ptr cloud_filtered_blob (new sensor_msgs::PointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>);


	pcl::toROSMsg<pcl::PointXYZ>(*currentCloud, *cloud_blob);

	std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	sor.setInputCloud (cloud_blob);
	sor.setLeafSize (0.01, 0.01, 0.01);
	sor.filter (*cloud_filtered_blob);

	// Convert to the templated PointCloud
	pcl::fromROSMsg (*cloud_filtered_blob, *cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

	// Write the downsampled version to disk
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("box_downsampled.pcd", *cloud_filtered, false);

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
	// While 30% of the original cloud is still there
	while (cloud_filtered->points.size () > 0.3 * nr_points)
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
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ConvexHull<pcl::PointXYZ> chull;
		chull.setInputCloud (cloud_p);
		chull.reconstruct (*cloud_hull);

		std::stringstream ss;
		ss << "box_plane_" << i << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

		std::stringstream ss2;
		ss2 << "box_plane_hull_" << i << ".pcd";
		writer.write<pcl::PointXYZ> (ss2.str (), *cloud_hull, false);
		//viewer.showCloud(cloud_p);
		// Create the filtering object
		extract.setNegative (true);

		//FIX
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_temp (new pcl::PointCloud<pcl::PointXYZ>(*cloud_filtered));
		extract.filter (*cloud_filtered_temp);
		cloud_filtered = cloud_filtered_temp;

		i++;
	}
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
		break;
		case'p':
			drawPC = !drawPC;
			break;
	
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
		case '<':		
		case ',':		
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
		case '-':		
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
		case 'o':
			kinect.setCameraTiltAngle(angle);	// go back to prev tilt
			kinect.open();
			break;
		case 'c':
			kinect.setCameraTiltAngle(0);		// zero the tilt
			kinect.close();
			break;

		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;

		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;

		case 's':
			saveCloud();
			break;
		case 'b':
			processBlobsClouds();
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
void testApp::mouseMoved(int x, int y) {
	pointCloudRotationY = x;
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}


