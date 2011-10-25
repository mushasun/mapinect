#include <ax12.h>

AX12 motor1;
AX12 motor2;
AX12 motor4;
AX12 motor8;

void setup() 
{
  motor1 = AX12();
  motor2 = AX12();
  motor4 = AX12();

  Serial.begin (9600);  // inicializa el Serial a 115,2 Kb/s
  AX12::init (1000000);   // inicializa los AX12 a 1 Mb/s

  byte detect;          
  //byte num = AX12::autoDetect (&detect, 1); // detección de IDs
  //Serial.println (num, DEC);
  motor1.id = 1;
  motor2.id = 2; // asigna las ID detectadas a los motores definidos previamente
  //motor2.SRL = RETURN_ALL;

  motor4.id = 4; // asigna las ID detectadas a los motores definidos previamente
//  motor4.SRL = RETURN_ALL;
  motor8.id = 8;

  motor1.setVel (7);
  delay(100);
  motor1.setPos (pasarAngulo(128));
  delay(100);
  motor2.setVel (7);
  delay(100);
  motor2.setPos (pasarAngulo(128));
  delay(100);
  motor4.setVel (7);
  delay(100);
  motor4.setPos (pasarAngulo(128));
  delay(100);
  motor8.setVel (15);
  delay(100);
  motor8.setPos (pasarAngulo(128));
  delay(100);
}

int pasarAngulo(byte angulo)
{
  /*byte mask = B10000000;
  int direccion = 1;
  if (mask & angulo)
  {
    direccion = -1;
  }
  mask = B01111111;
  angulo = mask & angulo ;*/
  int resultado = angulo * (1024.0 / 300.0) + 76;
  return resultado;
}

void loop()
{
  if (Serial.available() > 1)
  {
    byte id = Serial.read();
    byte angulo = Serial.read();
    int angulo_pasado = pasarAngulo(angulo);
    if (id==1){
      motor1.setVel (15);
      motor1.setPos (angulo_pasado);
    }
    if (id==2){
      motor2.setVel (15);
      motor2.setPos (angulo_pasado);
    }
    else if (id==4)
    {
      motor4.setVel (15);
      motor4.setPos (angulo_pasado);
    }
    else if (id==8)
    {
      motor8.setVel (50);
      motor8.setPos (angulo_pasado);
    }
    delay(100);
  } 
}
