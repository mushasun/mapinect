#include "lpmt.h"
#include "ofGraphicsUtils.h"
#include "ofxSimpleGuiToo.h"
#include "monitor.h"
#include "winUtils.h"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

#define		SLASH_STRING		"/"
#define		BACKSLASH_STRING	"\\"

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

void LPMT::setup(ofxKinect *kinect)
{
	this->kinect = kinect;

	//we run at 60 fps!
	//ofSetVerticalSync(true);

	// we scan the img dir for images
	string imgDir = string("./data/img");
	imgFiles = vector<string>();
	//getdir(imgDir,imgFiles);
	show_files(imgDir,imgFiles);
	int size = imgFiles.size();
	string *images = new string[size];
	for (unsigned int i = 0;i < imgFiles.size();i++) {
		string currImg = imgFiles[i];
		/*	if (currImg.find(".") != -1) {
		currImg.replace(currImg.find("."), 1, ""); 
		} */
		while (currImg.find(SLASH_STRING) != -1) {
			currImg.replace(currImg.find(SLASH_STRING), 1, BACKSLASH_STRING);	
		}	
		if (currImg.find_last_of(BACKSLASH_STRING) != -1) {
			imgFiles[i] = currImg.substr(currImg.find_last_of(BACKSLASH_STRING)+1,currImg.size());
		}
		images[i]= imgFiles[i];
	}


	// we scan the video dir for videos
	string videoDir = string("./data/video");
	videoFiles = vector<string>();
	//getdir(videoDir,videoFiles);
	show_files(videoDir,videoFiles);
	size = videoFiles.size();
	string *videos = new string[size];
	for (unsigned int i = 0;i < videoFiles.size();i++) {
		string currVideo = videoFiles[i];
		/* if (currVideo.find(".") != -1) {
		currVideo.replace(currVideo.find("."), 1, ""); 
		} */
		while (currVideo.find(SLASH_STRING) != -1) {
			currVideo.replace(currVideo.find(SLASH_STRING), 1, BACKSLASH_STRING);	
		}	
		if (currVideo.find_last_of(BACKSLASH_STRING) != -1) {
			videoFiles[i] = currVideo.substr(currVideo.find_last_of(BACKSLASH_STRING)+1,currVideo.size());
		}
		videos[i]= videoFiles[i];
	}

	// we scan the slideshow dir for videos
	string slideshowDir = string("./data/slideshow");
	slideshowFolders = vector<string>();
	//getdir(slideshowDir,slideshowFolders);
	show_files(slideshowDir,slideshowFolders);
	size = slideshowFolders.size();
	string *slideshows = new string[size];
	for (unsigned int i = 0;i < slideshowFolders.size();i++) {
		string currSlideShow = slideshowFolders[i];
		/* if (currSlideShow.find(".") != -1) {
		currSlideShow.replace(currSlideShow.find("."), 1, ""); 
		} */
		while (currSlideShow.find(SLASH_STRING) != -1) {
			currSlideShow.replace(currSlideShow.find(SLASH_STRING), 1, BACKSLASH_STRING);	
		}	
		if (currSlideShow.find_last_of(BACKSLASH_STRING) != -1) {
			slideshowFolders[i] = currSlideShow.substr(currSlideShow.find_last_of(BACKSLASH_STRING)+1,currSlideShow.size());
		}
		slideshows[i]= slideshowFolders[i];
	}


	ofSetLogLevel(OF_LOG_ERROR);
	ttf.loadFont("./type/frabk.ttf", 11);
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
	//camGrabber.initGrabber(camWidth,camHeight);

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

//--------------------------------------------------------------
void LPMT::update() {

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

//--------------------------------------------------------------
void LPMT::draw() {
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
	if (isSetup) {
		gui.draw();
	}
}

//--------------------------------------------------------------
void LPMT::keyPressed (int key) {
	switch (key) {
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
			int totalPixels = camWidth*camHeight*3;
			unsigned char * pixels = kinect->getPixels();
			snapshotTexture.loadData(pixels, camWidth,camHeight, GL_RGB);
		}
		break;
	case 'y':
		ofBeginCustomFullscreen(1280, 0, 1280, 768);
		break;
	case 'l':
		monitor::SetDisplayDefaults();
	}
}

//--------------------------------------------------------------
void LPMT::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void LPMT::mouseDragged(int x, int y, int button)
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
void LPMT::mousePressed(int x, int y, int button)
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
void LPMT::windowResized(int w, int h)
{

}

//--------------------------------------------------------------
void LPMT::mouseReleased(int x, int y, int button)
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

