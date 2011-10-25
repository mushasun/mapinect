#ifndef ARDUINO_H__
#define ARDUINO_H__

#include "ofArduino.h"
#include "ofSerial.h"

class Arduino: public ofArduino{
	
public:
	Arduino();

    virtual ~Arduino();

	void sendMotor(char value, int id);

	char* read();

};
#endif
