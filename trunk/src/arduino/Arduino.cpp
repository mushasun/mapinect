#include "Arduino.h"

Arduino::Arduino() : ofArduino()
{
	int a;//ejecuta el método padre
}

Arduino::~Arduino()
{
	_port.close();
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
	sendByte(id_char);
	sendByte(value);
}

char* Arduino::read()
{
	char* text = new char[_charHistory.size()];
	for (int i = 0; i < _charHistory.size(); i++){
		text[i] = _port.readByte();
	}
	return text;
}
