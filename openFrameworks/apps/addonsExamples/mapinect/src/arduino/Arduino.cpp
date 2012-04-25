#include "Arduino.h"

#include "ofxXmlSettings.h"

namespace mapinect {

#define		ARDUINO_CONFIG		"ArduinoConfig:"

#define		ID_MOTOR_1			1
#define		ID_MOTOR_2			2
#define		ID_MOTOR_4			4
#define		ID_MOTOR_8			8

#define		ANGLE_UNDEFINED		MAXCHAR
#define		ANGLE_DEFAULT		0
#define		KEY_UNDEFINED		""

#define		MOTOR_ANGLE_OFFSET	128

	static string	COM_PORT;
	static char		KEY_MOVE_1R;
	static char		KEY_MOVE_1L;
	static char		KEY_MOVE_2R;
	static char		KEY_MOVE_2L;
	static char		KEY_MOVE_4R;
	static char		KEY_MOVE_4L;
	static char		KEY_MOVE_8R;
	static char		KEY_MOVE_8L;
	static char		KEY_RESET;
	static char		KEY_PRINT_STATUS;
	static char		RESET_ANGLE1;
	static char		RESET_ANGLE2;
	static char		RESET_ANGLE4;
	static char		RESET_ANGLE8;
	static int		ANGLE_STEP;

	Arduino::Arduino()
	{
	}

	Arduino::~Arduino()
	{
		serial.close();
	}

	bool Arduino::setup()
	{
		ofxXmlSettings XML;
		if(XML.loadFile("Arduino_Config.xml")) {

			COM_PORT = XML.getValue(ARDUINO_CONFIG "COM_PORT", "COM3");

			KEY_MOVE_1R = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_1R", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_1L = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_1L", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_2R = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_2R", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_2L = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_2L", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_4R = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_4R", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_4L = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_4L", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_8R = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_8R", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_8L = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_8L", KEY_UNDEFINED).c_str()[0];

			KEY_RESET = XML.getValue(ARDUINO_CONFIG "KEY_RESET", KEY_UNDEFINED).c_str()[0];
			KEY_PRINT_STATUS = XML.getValue(ARDUINO_CONFIG "KEY_PRINT_STATUS", KEY_UNDEFINED).c_str()[0];

			ANGLE_STEP = XML.getValue(ARDUINO_CONFIG "ANGLE_STEP", 2);

			RESET_ANGLE1 = XML.getValue(ARDUINO_CONFIG "RESET_ANGLE1", ANGLE_DEFAULT);
			RESET_ANGLE2 = XML.getValue(ARDUINO_CONFIG "RESET_ANGLE2", ANGLE_DEFAULT);
			RESET_ANGLE4 = XML.getValue(ARDUINO_CONFIG "RESET_ANGLE4", ANGLE_DEFAULT);
			RESET_ANGLE8 = XML.getValue(ARDUINO_CONFIG "RESET_ANGLE8", ANGLE_DEFAULT);
		}

		serial.enumerateDevices();
		if (serial.setup(COM_PORT, 9600)) {
			reset();
			return true;
		}
		return false;
	}

	void Arduino::exit() {
		serial.close();
	}

	void Arduino::update() {

	}

	void Arduino::draw() {

	}

	void Arduino::keyPressed (int key) {

		if (key == KEY_MOVE_1R) {
			angleMotor1 += ANGLE_STEP;
			sendMotor((char) angleMotor1, ID_MOTOR_1);
		}
		else if (key == KEY_MOVE_1L) {
			angleMotor1 -= ANGLE_STEP;
			sendMotor((char) angleMotor1, ID_MOTOR_1);
		}
		else if (key == KEY_MOVE_2R) {
			angleMotor2 += ANGLE_STEP;
			sendMotor((char) angleMotor2, ID_MOTOR_2);
		}
		else if (key == KEY_MOVE_2L) {
			angleMotor2 -= ANGLE_STEP;
			sendMotor((char) angleMotor2, ID_MOTOR_2);
		}
		else if (key == KEY_MOVE_4R) {
			angleMotor4 += ANGLE_STEP;
			sendMotor((char) angleMotor4, ID_MOTOR_4);
		}
		else if (key == KEY_MOVE_4L) {
			angleMotor4 -= ANGLE_STEP;
			sendMotor((char) angleMotor4, ID_MOTOR_4);
		}
		else if (key == KEY_MOVE_8R) {
			angleMotor8 += ANGLE_STEP;
			sendMotor((char) angleMotor8, ID_MOTOR_8);
		}
		else if (key == KEY_MOVE_8L) {
			angleMotor8 -= ANGLE_STEP;
			sendMotor((char) angleMotor8, ID_MOTOR_8);
		}
		else if (key == KEY_RESET) {
			reset();
		}
		else if (key == KEY_PRINT_STATUS) {
			cout << read() << endl;
		}
	}


	void Arduino::reset()
	{
		if (RESET_ANGLE1 != ANGLE_UNDEFINED) {
			angleMotor1 = RESET_ANGLE1;
			sendMotor((char) angleMotor1, ID_MOTOR_1);
		}
		if (RESET_ANGLE2 != ANGLE_UNDEFINED) {
			angleMotor2 = RESET_ANGLE2;
			sendMotor((char) angleMotor2, ID_MOTOR_2);
		}
		if (RESET_ANGLE4 != ANGLE_UNDEFINED) {
			angleMotor4 = RESET_ANGLE4;
			sendMotor((char) angleMotor4, ID_MOTOR_4);
		}
		if (RESET_ANGLE8 != ANGLE_UNDEFINED) {
			angleMotor8 = RESET_ANGLE8;
			sendMotor((char) angleMotor8, ID_MOTOR_8);
		}
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
		value += MOTOR_ANGLE_OFFSET;
		cout << my_byte_to_binary((int)value) <<endl;
		char id_char = (char) id;
		serial.writeByte(id_char);
		serial.writeByte(value);
	}

	string Arduino::read()
	{
		string result("");
		int cantidad_bytes = serial.available();
		if (cantidad_bytes) {
			unsigned char lectura = 0;
			int i = 0;
			while(serial.readBytes(&lectura, 1) > 0) {
				result[i] = lectura;
				i++;
			}
			result[i] = 0;
		}
		return result;
	}
}
