#include "ofxVecUtils.h"
#include "ofMain.h"
#include "Segment3D.h"

class Tejo
{
public:

	
	Tejo(){}
	Tejo(ofxVec3f position, double radio, ofxVec3f direction, double vel, ofxVec3f tableNorm, ofxVec3f tableCenter);
	~Tejo(){}

	inline double getRadio() { return radio; }
	inline ofxVec3f getPos() { return position; }

	void draw();
	void update(std::vector<Segment3D> segmentos);

private:
		ofxVec3f position;
		ofxVec3f direction;
		double vel;
        double radio;
        void move();
        bool check_collision(std::vector<Segment3D> segmentos);
        //bool hayInterseccionEsfera(vector<Esfera*> cuerpos);
        static float AbsorcionEnergeticaEnChoque ;
        int idBalaTextura;
        float moduloVectorVelocidad;

		ofxVec3f tableNormal;
		ofxVec3f tableCenter;
};