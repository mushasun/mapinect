#include "VisualizerApp.h"

#include "ofxFensterManager.h"

namespace visualizerApp {

	class visualizerWindow: public ofxFensterListener {
	public:
		visualizerWindow(ofxFenster* window) : window(window) { }

		~visualizerWindow() {
			cout << "Image Listener destroyed" << endl;
		}
		void setup() {
			// load song
			music.loadSound("sounds/miles.mp3");
			music.setVolume(1.0f);
			music.play();
			music.setLoop(true);

			vis.setup();
		}
		void draw() {
			ofSetColor(255);
			vis.draw(ofVec3f(0,0,0),
				ofVec3f(window->getWidth(),0,0),
				ofVec3f(window->getWidth(),window->getHeight(),0),
				ofVec3f(0,window->getHeight(),0));
		}
		void update() {
			ofSoundUpdate();
			vis.update();
		}

		void keyReleased(int key, ofxFenster* window) {
			if(key==' ')
				ofxFensterManager::get()->deleteFenster(window);
			if(key == 'm'){ //this is to test set and get window position
				ofPoint winPos = window->getWindowPosition();
				window->setWindowPosition(winPos.x + 10, winPos.y);
			}
			else if (key == 'f')
			{
				window->setFullscreen(window->getWindowMode() != OF_FULLSCREEN);
			}
		}

        ofSoundPlayer music;

		Visualizer vis;
		ofxFenster* window;
	};

	//--------------------------------------------------------------
	VisualizerApp::VisualizerApp() {
	}

	//--------------------------------------------------------------
	VisualizerApp::~VisualizerApp() {
	}

	//--------------------------------------------------------------
	void VisualizerApp::setup() {
		// load song
		/*
		music.loadSound("sounds/miles.mp3");
		music.setVolume(1.0f);
		music.play();
		music.setLoop(true);
		*/

		vis.setup();
		
		ofxFenster* win=ofxFensterManager::get()->createFenster(400, 0, 400, 400, OF_WINDOW);
		ofxFensterListener* visualizer = new visualizerWindow(win);
		win->addListener(visualizer);
		win->setBackgroundColor(ofRandom(255), ofRandom(255), ofRandom(255));
		visualizer->setup();
	}

	//--------------------------------------------------------------
	void VisualizerApp::exit() {
	}

	//--------------------------------------------------------------
	void VisualizerApp::debugDraw()
	{
		
		//drawVisualizer();
	}

	//--------------------------------------------------------------
	void VisualizerApp::draw()
	{
		vis.draw(ofVec3f(0,0,0),ofVec3f(300,0,0),ofVec3f(300,300,0),ofVec3f(0,300,0));
		drawVisualizer();

		ofSetColor(255);
		ofDrawBitmapString(ofToString(ofGetFrameRate()),20,20);

	}

	void VisualizerApp::drawVisualizer()
	{
	}
	//--------------------------------------------------------------
	void VisualizerApp::update() {
		ofSoundUpdate();
		vis.update();
	}

	//--------------------------------------------------------------
	void VisualizerApp::keyPressed(int key)
	{
	}

	//--------------------------------------------------------------
	void VisualizerApp::mouseMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void VisualizerApp::mouseDragged(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void VisualizerApp::mousePressed(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void VisualizerApp::mouseReleased(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void VisualizerApp::windowResized(int w, int h)
	{
	}

}
