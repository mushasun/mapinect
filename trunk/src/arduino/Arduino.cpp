#include "Arduino.h"

Arduino::Arduino()
{
	serial.enumerateDevices();
	serial.setup("COM3", 9600); 
}

Arduino::~Arduino()
{
	serial.close();
}

const char *my_byte_to_binary(int x)
{
    static char b[9];
    b[0] = '\0';

    int z;
    for (z = 256; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}


void Arduino::sendMotor(char value, int id)
{
	cout << my_byte_to_binary((int)value) <<endl;
	char id_char = (char) id;
	serial.writeByte(id_char);
	serial.writeByte(value);
}

unsigned char* Arduino::read()
{
	int cantidad_bytes = serial.available();
	if (cantidad_bytes){
		unsigned char* lectura = new unsigned char[cantidad_bytes];
		memset(lectura, 0, cantidad_bytes);

		int i = 0;
	
		while(serial.readBytes(&lectura[i], 1) > 0){
			i++;
		};

		cout << lectura;

		return lectura;
	}
}
