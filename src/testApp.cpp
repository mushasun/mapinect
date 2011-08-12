#include "testApp.h"
#include "pointUtils.h"
#include "ofxVecUtils.h"
#include "Line2D.h"
#include "Triangle2D.h"
#include "ofxSimpleGuiToo.h"

#define DEFAULT_NAME		"test"
#define PCD_EXTENSION		".pcd"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

using namespace std;
using namespace boost::filesystem;
//--------------------------------------------------------------


void show_files( const path directory, vector<string> &files)
{
	if( exists( directory ) )
	{		
		boost::filesystem::directory_iterator end ;
		for( boost::filesystem::directory_iterator iter(directory) ; iter != end ; ++iter ) 
		{
			if ( boost::filesystem::is_regular_file(iter->status()) )
			{
				boost::filesystem::path p;
				p = iter->path();
				string str = p.string();
				if (str != "." && str !="..") 
				{
					files.push_back(p.string());
				}
			}
		}			
	}
}

void testApp::cargar_lpmt()
{
//we run at 60 fps!
    ofSetVerticalSync(true);

    // we scan the img dir for images
	string imgDir = string("../data/img");
    imgFiles = vector<string>();
    //getdir(imgDir,imgFiles);
	show_files(imgDir,imgFiles);
	int size = imgFiles.size();
    string *images = new string[size];
    for (unsigned int i = 0;i < imgFiles.size();i++) {
        images[i]= imgFiles[i];
    }


    // we scan the video dir for videos
	string videoDir = string("../data/video");
    videoFiles = vector<string>();
    //getdir(videoDir,videoFiles);
    show_files(videoDir,videoFiles);
	size = videoFiles.size();
    string *videos = new string[size];
    for (unsigned int i = 0;i < videoFiles.size();i++) {
        videos[i]= videoFiles[i];
    }

    // we scan the slideshow dir for videos
	string slideshowDir = string("../data/slideshow");
    slideshowFolders = vector<string>();
    //getdir(slideshowDir,slideshowFolders);
    show_files(slideshowDir,slideshowFolders);
	size = slideshowFolders.size();
	string *slideshows = new string[size];
    for (unsigned int i = 0;i < slideshowFolders.size();i++) {
        slideshows[i]= slideshowFolders[i];
    }


    ttf.loadFont("../type/frabk.ttf", 11);
    // set border color for quads in setup mode
    borderColor = 0x666666;
    // starts in quads setup mode
    isSetup = true;
    // starts in windowed mode
    bFullscreen	= 0;
    // gui is on at start
    bGui = 1;
    ofSetWindowShape(800, 600);

    // camera stuff
    camWidth = 640;	// try to grab at this size.
    camHeight = 480;
	
    camGrabber.setVerbose(true);
    camGrabber.initGrabber(camWidth,camHeight);

    // texture for snapshot background
    snapshotTexture.allocate(camWidth,camHeight, GL_RGB);
    snapshotOn = 0;

    // initializes layers array
    for(int i = 0; i < 36; i++) {
    layers[i] = -1;
    }


    // defines the first 4 default quads
    quads[0].setup(0.0,0.0,0.5,0.0,0.5,0.5,0.0,0.5,imgFiles, videoFiles, slideshowFolders);
    quads[0].quadNumber = 0;
    quads[1].setup(0.5,0.0,1.0,0.0,1.0,0.5,0.5,0.5,imgFiles, videoFiles, slideshowFolders);
    quads[1].quadNumber = 1;
    quads[2].setup(0.0,0.5,0.5,0.5,0.5,1.0,0.0,1.0,imgFiles, videoFiles, slideshowFolders);
    quads[2].quadNumber = 2;
    quads[3].setup(0.5,0.5,1.0,0.5,1.0,1.0,0.5,1.0,imgFiles, videoFiles, slideshowFolders);
    quads[3].quadNumber = 3;
    // define last one as active quad
    activeQuad = 3;
    // number of total quads, to be modified later at each quad insertion
    nOfQuads = 4;
    layers[0] = 0;
    quads[0].layer = 0;
    layers[1] = 1;
    quads[1].layer = 1;
    layers[2] = 2;
    quads[2].layer = 2;
    layers[3] = 3;
    quads[3].layer = 3;



    // GUI STUFF ---------------------------------------------------

    gui.addTitle("show/hide quads");
    // overriding default theme
    gui.config->toggleHeight = 18;
    gui.config->sliderTextHeight = 22;
    gui.config->titleHeight = 18;
    gui.config->fullActiveColor = 0x6B404B;

    // adding controls
    // first a general page for toggling layers on/off
    for(int i = 0; i < 36; i++)
    {
    gui.addToggle("quad "+ofToString(i), quads[i].isOn);
    }

    // then two pages of settings for each instantiable layer
    for(int i = 0; i < 36; i++)
    {
		gui.addPage("quad "+ofToString(i)+" - 1/3");
		gui.addTitle("quad n. "+ofToString(i));
		gui.addToggle("show/hide", quads[i].isOn);
		gui.addToggle("img bg on/off", quads[i].imgBg);
		gui.addComboBox("image bg", quads[i].bgImg, imgFiles.size(), images);
		gui.addSlider("img mult X", quads[i].imgMultX, 0.2, 4.0);
		gui.addSlider("img mult Y", quads[i].imgMultY, 0.2, 4.0);
		gui.addColorPicker("img colorize", &quads[i].imgColorize.r);
		gui.addTitle("Solid color").setNewColumn(true);
		gui.addToggle("solid bg on/off", quads[i].colorBg);
		gui.addColorPicker("Color", &quads[i].bgColor.r);
		gui.addToggle("transition color", quads[i].transBg);
		gui.addColorPicker("second Color", &quads[i].secondColor.r);
		gui.addSlider("trans duration", quads[i].transDuration, 0.2, 60.0);

		gui.addPage("quad "+ofToString(i)+" - 2/3");
		gui.addTitle("Video");
		gui.addToggle("video bg on/off", quads[i].videoBg);
		gui.addComboBox("video bg", quads[i].bgVideo, videoFiles.size(), videos);
		gui.addSlider("video mult X", quads[i].videoMultX, 0.2, 4.0);
		gui.addSlider("video mult Y", quads[i].videoMultY, 0.2, 4.0);
		gui.addColorPicker("video colorize", &quads[i].videoColorize.r);
		gui.addSlider("video sound vol", quads[i].videoVolume, 0, 100);
		gui.addSlider("video speed", quads[i].videoSpeed, -2.0, 4.0);
		gui.addToggle("video loop", quads[i].videoLoop);
		gui.addTitle("Camera bg").setNewColumn(true);
		gui.addToggle("cam on/off", quads[i].camBg);
		gui.addSlider("camera mult X", quads[i].camMultX, 0.2, 4.0);
		gui.addSlider("camera mult Y", quads[i].camMultY, 0.2, 4.0);
		gui.addColorPicker("cam colorize", &quads[i].camColorize.r);
		gui.addTitle("Greenscreen");
		gui.addSlider("g-screen threshold", quads[i].thresholdGreenscreen, 0, 120);
		gui.addColorPicker("greenscreen col", &quads[i].colorGreenscreen.r);
		gui.addToggle("video greenscreen", quads[i].videoGreenscreen);
		gui.addToggle("camera greenscreen", quads[i].camGreenscreen);
		gui.addTitle("Slideshow");
		gui.addToggle("slideshow on/off", quads[i].slideshowBg);
		//gui.addComboBox("slideshow folder", quads[i].bgSlideshow, slideshowFolders.size(), slideshows);
		gui.addSlider("slide duration", quads[i].slideshowSpeed, 0.1, 15.0);
		gui.addToggle("slides to quad size", quads[i].slideFit);
		gui.addToggle("keep aspect ratio", quads[i].slideKeepAspect);

		gui.addPage("quad "+ofToString(i)+" - 3/3");
		gui.addTitle("Corner 0");
		gui.addSlider("X", quads[i].corners[0].x, -1.0, 2.0);
		gui.addSlider("Y", quads[i].corners[0].y, -1.0, 2.0);
		gui.addTitle("Corner 3");
		gui.addSlider("X", quads[i].corners[3].x, -1.0, 2.0);
		gui.addSlider("Y", quads[i].corners[3].y, -1.0, 2.0);
		gui.addTitle("Corner 1").setNewColumn(true);
		gui.addSlider("X", quads[i].corners[1].x, -1.0, 2.0);
		gui.addSlider("Y", quads[i].corners[1].y, -1.0, 2.0);
		gui.addTitle("Corner 2");
		gui.addSlider("X", quads[i].corners[2].x, -1.0, 2.0);
		gui.addSlider("Y", quads[i].corners[2].y, -1.0, 2.0);
    }

    // then we set displayed gui page to the one corresponding to active quad and show the gui
    gui.setPage((activeQuad*3)+2);
    gui.show();
}
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
	octree = new octree::OctreePointCloudChangeDetector<PointXYZ>(OCTREE_RES); //Valor de resolucion sacado del ejemplo

	timer = 0;
	baseCloudSetted = false;
	drawCalibration = true;

	//agregando lo LPMT
	cargar_lpmt();
}

//--------------------------------------------------------------
void testApp::update() {
	if (showlpmt)
	{
		// grabs video frame from camera and passes pixels to quads
		camGrabber.grabFrame();
		if (camGrabber.isFrameNew()){
			int totalPixels = camWidth*camHeight*3;
			unsigned char * pixels = camGrabber.getPixels();
			for (int j = 0; j < 36; j++) {
				int i = layers[j];
				if ((i != -1) && (quads[i].initialized)) {
					if (quads[i].camBg) {
					quads[i].camPixels = pixels;
					quads[i].camWidth = camWidth;
					quads[i].camHeight = camHeight;
					}
				}
			}
		}


		// sets default window background, grey in setup mode and black in projection mode
		if (isSetup) {
		ofBackground(20, 20, 20);
		}
		else {
		ofBackground(0, 0, 0);
		}
		//ofSetWindowShape(800, 600);
		// loops through initialized quads and runs update, setting the border color as well
		for(int j = 0; j < 36; j++)
		{
			int i = layers[j];
			if ((i != -1) && (quads[i].initialized))
			{
				quads[i].update();
				quads[i].borderColor = borderColor;
			}
		}
	
	}
	else
	{
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

			//update the cv image
			grayImage.flagImageChanged();
	
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
    	// also, find holes is set to true so we will get interior contours as well....
    	//contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);

			//for(int i = 0 ; i < contourFinder.nBlobs; i++)
			//{
			//	cout << "min: (" << contourFinder.blobs[i].boundingRect.x << ", " << contourFinder.blobs[i].boundingRect.x + contourFinder.blobs[i].boundingRect.y << ")" << endl;
			//	cout << "max: (" << contourFinder.blobs[i].boundingRect.x + contourFinder.blobs[i].boundingRect.width << ", " << contourFinder.blobs[i].boundingRect.x + contourFinder.blobs[i].boundingRect.y + contourFinder.blobs[i].boundingRect.height<< ")" << endl;
			//}

			if(!baseCloudSetted)
			{
				setInitialPointCloud();
				baseCloudSetted = true;
				cout << "Base cloud setted..." << endl;
			}
			else if(ofGetElapsedTimef() - timer > 5){
				cout << "Process diferences" << endl;
				//processDiferencesClouds();
				timer = ofGetElapsedTimef();
			}
		}
		/*
		Se procesa la diferencia apretando el espacio.
		else if(ofGetElapsedTimef() - timer > 5){
			cout << "Process diferences" << endl;
			processDiferencesClouds();
			timer = ofGetElapsedTimef();
		}*/
	}
}

//--------------------------------------------------------------
void testApp::draw() {
	if (showlpmt)
	{
		// in setup mode sets active quad border to be white
		if (isSetup)
		{
			quads[activeQuad].borderColor = 0xFFFFFF;
			// if snapshot is on draws it as window background
			if (snapshotOn) {
			ofEnableAlphaBlending();
			ofSetColor(0xFFFFFF);
			snapshotTexture.draw(0,0,ofGetWidth(),ofGetHeight());
			ofDisableAlphaBlending();
			}
		}
		// loops through initialized quads and calls their draw function
		for(int j = 0; j < 36; j++)
		{
			int i = layers[j];
			if ((i != -1) && (quads[i].initialized))
			{
				quads[i].draw();
			}
		}



		// in setup mode writes the number of active quad at the bottom of the window
		if (isSetup)
		{
			ofSetColor(0xFFFFFF);
			ttf.drawString("active quad: "+ofToString(activeQuad), 30, ofGetHeight()-25);
		}

	   // draws gui
	   if (isSetup)
	   {
			gui.draw();
	   }
	}
	else
	{
		ofSetColor(255, 255, 255);
		if(drawPC){
			ofPushMatrix();
			ofTranslate(420, 320);
			// we need a proper camera class
			drawPointCloud();
			ofPopMatrix();
		}
		else if(drawCalibration)
		{
			if(drawDepth)
				kinect.drawDepth(0,0,640,480);

			//Dibujo nube de diferencia
			//int points = diffCloud->size();
			//cout << "diff_size = " << points << endl;
			//glBegin(GL_POINTS);
			//	for(int j = 0; j < points; j ++) {
			//		ofPoint cur (diffCloud->at(j).x, diffCloud->at(j).y, 0);
			//		cur += ofPoint(320,240,0);
			//		ofColor color = kinect.getCalibratedColorAt(cur);
			//		//ofSetColor(color.r,color.g,color.b);
			//		ofSetColor(0,255,0);
			//		glVertex3f(cur.x, cur.y, cur.z);
			//	}
			//glEnd();

			////Dibujo Hull
			//points = vCloudHull.size();
			//glPointSize(2.0);
			//glBegin(GL_POINTS);
			//for(int j = 0; j < points; j ++) {
			//		ofPoint cur (vCloudHull.at(j).x, vCloudHull.at(j).y, 0);
			//		glColor3ub(255,0,0);
			//		glVertex3f(cur.x + 320, cur.y + 240, cur.z);
			//}
			//glEnd();


			ofEnableAlphaBlending();
		
			ofSetColor(10,200,0,200);
			ofPushMatrix();
			ofTranslate(320,240,0);
			for(int i = 0; i < detectedPlanes; i++){
				//Dibujo plano
				PointCloud<PointXYZ> plane = planes[i];
				int points = plane.size();
				glBegin(GL_POINTS);
					for(int j = 0; j < points; j ++) {
						ofPoint cur (plane.at(j).x, plane.at(j).y, 0);
						//cur += ofPoint(320,240,0);
						//ofColor color = kinect.getCalibratedColorAt(cur);
						//ofSetColor(color.r,color.g,color.b);
						ofSetColor(255,i*100,0);
						glVertex3f(cur.x, cur.y, cur.z);
					}
				glEnd();
			}
			ofPopMatrix();
		
			ofDisableAlphaBlending();
		}
		else{
			kinect.drawDepth(10, 10, 400, 300);
			kinect.draw(420, 10, 400, 300);
			ofPushMatrix();
				ofTranslate(420 + 200, 10 + 150);
				glColor3f(0.0, 1.0, 0.0);
				glBegin(GL_TRIANGLES);
					glVertex3f(detectedPlane.getVA().x, detectedPlane.getVA().y, 3);
					glVertex3f(detectedPlane.getVB().x, detectedPlane.getVB().y, 3);
					glVertex3f(detectedPlane.getVC().x, detectedPlane.getVC().y, 3);
					glVertex3f(detectedPlane.getVA().x, detectedPlane.getVA().y, 3);
					glVertex3f(detectedPlane.getVB().x, detectedPlane.getVB().y, 3);
					glVertex3f(detectedPlane.getVD().x, detectedPlane.getVD().y, 3);
				glEnd();

				glBegin(GL_POLYGON);
					ofCircle(detectedPlane.getVA().x, detectedPlane.getVA().y, 4);
					ofCircle(detectedPlane.getVB().x, detectedPlane.getVB().y, 4);
					ofCircle(detectedPlane.getVC().x, detectedPlane.getVC().y, 4);
					ofCircle(detectedPlane.getVD().x, detectedPlane.getVD().y, 4);
				glEnd();

				glColor3f(1.0, 0.0, 0.0);
				glBegin(GL_POINTS);
					for (int i = 0; i < vCloudHull.size(); i++) {
						glVertex3f(vCloudHull[i].x, vCloudHull[i].y, 5);
					}
				glEnd();

				glColor3f(1.0, 1.0, 1.0);
			ofPopMatrix();

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
}

void testApp::drawPointCloud() {
	ofScale(400, 400, 400);
	int w = 640;
	int h = 480;
	
	ofRotateY(pointCloudRotationY);
	float* distancePixels = kinect.getDistancePixels();
	

	for (int j = 0; j < 800; j++) {
		glBegin(GL_QUADS);
			glVertex3f(detectedPlane.getVA().x, detectedPlane.getVA().y, 400 - j);
			glVertex3f(detectedPlane.getVB().x, detectedPlane.getVB().y, 400 - j);
			glVertex3f(detectedPlane.getVC().x, detectedPlane.getVC().y, 400 - j);
			glVertex3f(detectedPlane.getVD().x, detectedPlane.getVD().y, 400 - j);
		glEnd();
	}
	
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
	//kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void testApp::saveCloud(const string& name){
	cout << "saving: " << name << "..." << endl;

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

	string filename = name + PCD_EXTENSION;
	pcl::io::savePCDFileASCII (filename, *cloud);
	std::cerr << "Saved " << cloud->points.size () << " data points to " << filename << std::endl;
}

void testApp::savePartialCloud(ofPoint min, ofPoint max, int id, const string& name){
	//Calcular tamaño de la nube
	cout << "saving: " << name << "..." << endl;
	PointCloud<PointXYZ>::Ptr partialColud = getPartialCloud(min,max);
	std::stringstream ss;
	ss << name << "-" << id << PCD_EXTENSION;
	pcl::io::savePCDFileASCII (ss.str(), *partialColud);
	std::cerr << "Saved " << partialColud->points.size () << " data points to " << ss.str() << std::endl;
}

PointCloud<PointXYZ>* testApp::loadCloud(const string& name) {
	cout << "loading: "<< name << "..."<<endl;
	pcl::PointCloud<pcl::PointXYZ>* tmpCloud = new pcl::PointCloud<PointXYZ>();
	string filename = name + PCD_EXTENSION;
	pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *tmpCloud);
	cout << name << " loaded!"<<endl;
	return tmpCloud;
}

PointCloud<PointXYZ>::Ptr testApp::getCloud(){
	return getPartialCloud(ofPoint(0,0),ofPoint(KINECT_WIDTH,KINECT_HEIGHT));
}

PointCloud<PointXYZRGB>::Ptr testApp::getColorCloud(){
	return getPartialColorCloud(ofPoint(0,0),ofPoint(KINECT_WIDTH,KINECT_HEIGHT));
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
					//ofxVec3f point = kinect.getWorldCoordinateFor(u,v);
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
PointCloud<PointXYZRGB>::Ptr testApp::getPartialColorCloud(ofPoint min, ofPoint max){
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

	register float* depth_map = kinect.getDistancePixels();
	unsigned char* color_map = kinect.getCalibratedRGBPixels();
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
					ofColor c = kinect.getColorAt(absU,absV);
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

void testApp::captureBlobsClouds(){
	for (int i = 0; i < contourFinder.nBlobs; i ++){
		ofxCvBlob blob = contourFinder.blobs[i];
		ofPoint min (blob.boundingRect.x,blob.boundingRect.y);
		ofPoint max (blob.boundingRect.x + blob.boundingRect.width ,blob.boundingRect.y + blob.boundingRect.height);

		savePartialCloud(min,max,i,DEFAULT_NAME);
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

PointCloud<PointXYZ>::Ptr testApp::getDifferenceIdx(bool &dif, int noise_filter){
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

void testApp::processDiferencesClouds(){
	start = clock();
	bool dif;
	PointCloud<PointXYZ>::Ptr filteredCloud = getDifferenceIdx(dif);
	if(dif)
		detectPlanes(filteredCloud);
	else
		cout << "No differences founded.";
	end = clock();

	printTime();
}

void testApp::printTime(){
	cout <<"elapsed: "<< ( (end - start)/CLOCKS_PER_SEC ) << endl;
}

void testApp::icp(){
	kinect.setCameraTiltAngle(0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr initial_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr initial = getCloud();
	std::stringstream ss;
	//ss << "icp_initial.pcd";
	//pcl::io::savePCDFileASCII (ss.str(), *initial);

	std::cerr<<"Cambio!"<< endl;
	//getchar();
	kinect.setCameraTiltAngle(0);
	ofSleepMillis(1000);
	this->update();

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
	kinect.setCameraTiltAngle(0);
}
//--------------------------------------------------------------
void testApp::detectPlanes(PointCloud<PointXYZ>::Ptr currentCloud){
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
		for (int k = 0; k < cloud_hull->size(); k++) {
			vCloudHull.push_back(POINTXYZ_OFXVEC3F(cloud_hull->at(k)));
		}

		detectedPlane.findQuad(vCloudHull);

		cout << time(NULL) - now << endl;

		i++;
	}

	cout << "Detected planes: " << detectedPlanes << endl;


}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	// figure out how to put the window in the center:
	int screenW = ofGetScreenWidth();
	int screenH = ofGetScreenHeight();
	switch (key) {
		case ' ':
			processDiferencesClouds();
		break;
		case'p':
			drawPC = !drawPC;
			break;
		case't':
			showlpmt = !showlpmt;
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
			//kinect.setCameraTiltAngle(0);		// zero the tilt
			//kinect.close();
			icp();
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
		//case OF_KEY_LEFT:
		//	drawDepth = true;
		//	break;
		//case OF_KEY_RIGHT:
		//	drawDepth = false;
			break;
		case 's':
			saveCloud(DEFAULT_NAME);
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
		case '1':
			gui.setPage(1);
			break;
		// toggles gui
		case 'g':
			gui.toggleDraw();
			bGui = !bGui;
			break;
		case '[':
			gui.prevPage();
			break;
		case ']':
			gui.nextPage();
			break;
		case 'f':
			bFullscreen = !bFullscreen;
			if(!bFullscreen)
			{
				ofSetWindowShape(800, 600);
				ofSetFullscreen(false);
				ofSetWindowPosition(screenW/2-800/2, screenH/2-600/2);
			}
			else if(bFullscreen == 1)
			{
				ofSetFullscreen(true);
			}
			break;
		case '/' :
			if (isSetup)
			{
				isSetup = false;
				for(int i = 0; i < 36; i++)
				{
					if ((i != -1) && (quads[i].initialized))
					{
						quads[i].isSetup = false;
					}
				}
			}
			else
			{
				isSetup = true;
				for(int i = 0; i < 36; i++)
				{
					if ((i != -1) && (quads[i].initialized))
					{
						quads[i].isSetup = true;
					}
				}
			}
			break;
		case 'a':// adds a new quad in the middle of the screen
			if (isSetup)
			{
				if (nOfQuads < 36)
				{
					quads[nOfQuads].setup(0.25,0.25,0.75,0.25,0.75,0.75,0.25,0.75, imgFiles, videoFiles, slideshowFolders);
					quads[nOfQuads].quadNumber = nOfQuads;
					layers[nOfQuads] = nOfQuads;
					quads[nOfQuads].layer = nOfQuads;
					activeQuad = nOfQuads;
					++nOfQuads;
					gui.setPage((activeQuad*3)+2);
				}
			}
			break;
		
		case OF_KEY_RIGHT:
			if (isSetup)
			{
				activeQuad += 1;
				if (activeQuad > nOfQuads-1)
				{
					activeQuad = 0;
				}
			}
			gui.setPage((activeQuad*3)+2);
			break;
		case OF_KEY_LEFT:
			if (isSetup)
			{
				activeQuad -= 1;
				if (activeQuad < 0)
				{
					activeQuad = nOfQuads-1;
				}
			}
			gui.setPage((activeQuad*3)+2);
			break;
		case 'k':
			snapshotOn = !snapshotOn;
			if (snapshotOn == 1) {
				//camGrabber.grabFrame();
				int totalPixels = camWidth*camHeight*3;
				//unsigned char * pixels = camGrabber.getPixels();
				kinect.update();
				unsigned char * pixels = kinect.getPixels();
				snapshotTexture.loadData(pixels, camWidth,camHeight, GL_RGB);
			}
		}
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y) {
	pointCloudRotationY = x;
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{
	if (isSetup && !bGui)
	{

		float scaleX = (float)x / ofGetWidth();
		float scaleY = (float)y / ofGetHeight();

		if(whichCorner >= 0)
		{
			quads[activeQuad].corners[whichCorner].x = scaleX;
			quads[activeQuad].corners[whichCorner].y = scaleY;
		}

	}
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{
if (isSetup && !bGui)
	{
		float smallestDist = 1.0;
		whichCorner = -1;

		for(int i = 0; i < 4; i++)
		{
			float distx = quads[activeQuad].corners[i].x - (float)x/ofGetWidth();
			float disty = quads[activeQuad].corners[i].y - (float)y/ofGetHeight();
			float dist  = sqrt( distx * distx + disty * disty);

			if(dist < smallestDist && dist < 0.1)
			{
				whichCorner = i;
				smallestDist = dist;
			}
		}
	}
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}


void testApp::mouseReleased(int x, int y, int button)
{
	if (whichCorner >= 0) {
		// snap detection for near quads
		float smallestDist = 1.0;
		int snapQuad = -1;
		int snapCorner = -1;
		for (int i = 0; i < 36; i++) {
			if ( i != activeQuad && quads[i].initialized) {
				for(int j = 0; j < 4; j++) {
					float distx = quads[activeQuad].corners[whichCorner].x - quads[i].corners[j].x;
					float disty = quads[activeQuad].corners[whichCorner].y - quads[i].corners[j].y;
					float dist = sqrt( distx * distx + disty * disty);
					// to tune snapping change dist value inside next if statement
					if (dist < smallestDist && dist < 0.0075) {
						snapQuad = i;
						snapCorner = j;
						smallestDist = dist;
					}
				}
			}
		}
		if (snapQuad >= 0 && snapCorner >= 0) {
			quads[activeQuad].corners[whichCorner].x = quads[snapQuad].corners[snapCorner].x;
			quads[activeQuad].corners[whichCorner].y = quads[snapQuad].corners[snapCorner].y;
		}
	}
	whichCorner = -1;
}