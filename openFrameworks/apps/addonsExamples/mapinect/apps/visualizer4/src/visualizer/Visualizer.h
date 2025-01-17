#ifndef VISUALIZER_H__
#define VISUALIZER_H__

#include "Looper.h"

class Visualizer {
  

#define WIDTH 1024
#define HEIGHT 768
#define HALF_WIDTH 512
#define HALF_HEIGHT 384

#define PINK 196, 0, 118
#define PURPLE 76, 0, 183
#define TEAL 24, 178, 198
#define BLUE 10, 144, 209

#define NUM_STARS 100
#define STAR_ALPHA 0.5

public:    
	Visualizer(){}
	void setup(vector<ofColor> baseColors);
	void update();
    void draw(ofVec3f v1, ofVec3f v2, ofVec3f v3, ofVec3f v4, bool bgOnly = false); 

	inline ofTexture getBgTexture(){ return bgFbo.getTextureReference(); }
    
private:
    void init();

	// Audio
    float avgSound;        
    float* fftSmoothed;
    int nBandsToGet;
    
    // Drawing
    ofFbo bgFbo, scene, final;
    static ofImage* texture;
	static ofImage* noise;
    static ofShader* texturizer;
    float blurScale;
    
    // Particles
    ofPolyline circle, tracer, wave;
    vector<ofPoint>stars;
    ofVbo starVbo;
    ofFbo starFbo;
    vector<Looper> loopers;
    
    // Interpolated values
    float count;
    float roll;
    ofColor background;
    ofColor rainbow;    
    float hue;
    ofColor interp;
    float interpAmt;
    
	vector<ofColor> colors;
};

#endif