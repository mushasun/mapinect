#ifndef ARDUINO_H__
#define ARDUINO_H__

#include "ofArduino.h"
#include "ofSerial.h"

class Arduino{
	
public:
	Arduino();

    virtual ~Arduino();

	void sendMotor(char value, int id);

	unsigned char* read();

	ofSerial	serial;

};
#endif
