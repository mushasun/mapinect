#include <ax12.h>

AX12 motor1;
AX12 motor2;
AX12 motor4;
AX12 motor8;

int posicion1 = 512;
int posicion2 = 512;
int posicion4 = 512;
int posicion8 = 512;

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

  motor1.setVel (10);
  delay(100);
  motor1.setPos (posicion1);
  delay(100);
  motor2.setVel (10);
  delay(100);
  motor2.setPos (posicion2);
  delay(100);
  motor4.setVel (10);
  delay(100);
  motor4.setPos (posicion4);
  delay(100);
  motor8.setVel (15);
  delay(100);
  motor8.setPos (posicion8);
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

void imprimirInfoMotores()
{
  /*Serial.println("Motor 1:");
  int pos = motor1.getPos(); 
  int load = motor1.getLoad();
  Serial.print("Posicion: ");
  Serial.println(pos, DEC);
  Serial.print("Carga: ");
  Serial.println(load, DEC);*/
  
  Serial.println("Motor 2:");
  int pos = motor2.getPos(); 
  int load = motor2.getLoad();
  Serial.print("Posicion: ");
  Serial.println(pos, BIN);
  Serial.print("Carga: ");
  Serial.println(load, DEC);
  
  Serial.println("Motor 4:");
  pos = motor4.getPos(); 
  load = motor4.getLoad();
  Serial.print("Posicion: ");
  Serial.println(pos, BIN);
  Serial.print("Carga: ");
  Serial.println(load, DEC);
  
  /*Serial.println("Motor 8:");
  pos = motor8.getPos(); 
  load = motor8.getLoad();
  Serial.print("Posicion: ");
  Serial.println(pos, DEC);
  Serial.print("Carga: ");
  Serial.println(load, DEC);*/
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
      posicion1 = angulo_pasado;
    }
    if (id==2){
      motor2.setVel (15);
      posicion2 = angulo_pasado;
    }
    else if (id==4)
    {
      motor4.setVel (15);
      posicion4 = angulo_pasado;
    }
    else if (id==8)
    {
      motor8.setVel (50);
      posicion8 = angulo_pasado;
    }
    imprimirInfoMotores();
  }
  delay(10);
  motor1.setPos (posicion1);
  delay(10);
  motor2.setPos (posicion2);
  delay(10);
  motor4.setPos (posicion4);
  delay(10);
  motor8.setPos (posicion8);
  delay(10);
}
