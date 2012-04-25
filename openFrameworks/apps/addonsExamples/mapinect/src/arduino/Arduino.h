#ifndef ARDUINO_H__
#define ARDUINO_H__

#include "ofSerial.h"

namespace mapinect {
	class Arduino{
	
	public:
		Arduino();
		virtual ~Arduino();

		virtual bool	setup();
		virtual void	exit();
		virtual void	update();
		virtual void	draw();

		virtual void	keyPressed(int key);

		void			reset();

	private:
		void			sendMotor(char value, int id);
		string			read();

		ofSerial		serial;
		signed int		angleMotor1;
		signed int		angleMotor2;
		signed int		angleMotor4;
		signed int		angleMotor8;

	};
}

#endif
