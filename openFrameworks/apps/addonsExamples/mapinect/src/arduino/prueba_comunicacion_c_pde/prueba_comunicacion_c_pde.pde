#include <ax12.h>

AX12 motor1;
AX12 motor2;
AX12 motor4;
AX12 motor8;

int posicion1 = 512;
int posicion2 = 512;
int posicion4 = 512;
int posicion8 = 570; //el cero está corrido :@

void setup() 
{
  motor1 = AX12();
  motor2 = AX12();
  motor4 = AX12();
  motor8 = AX12();

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

  delay(100);
  motor1.setPosVel (posicion1, 10);
  delay(100);
  motor2.setPosVel (posicion2, 10);
  delay(100);
  motor4.setPosVel (posicion4, 10);
  delay(100);
  motor8.setPosVel (posicion8, 10);
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
  Serial.println("Motor 1:");
  int pos = motor1.getPos(); 
  int load = motor1.getLoad();
  Serial.print("Posicion: ");
  Serial.println(pos, DEC);
  Serial.print("Carga: ");
  Serial.println(load, DEC);
  
  Serial.println("Motor 2:");
  pos = motor2.getPos(); 
  load = motor2.getLoad();
  Serial.print("Posicion: ");
  Serial.println(pos, DEC);
  Serial.print("Carga: ");
  Serial.println(load, DEC);
  
  Serial.println("Motor 4:");
  pos = motor4.getPos(); 
  load = motor4.getLoad();
  Serial.print("Posicion: ");
  Serial.println(pos, DEC);
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
      posicion1 = angulo_pasado;
    }
    if (id==2){
      posicion2 = angulo_pasado;
    }
    else if (id==4)
    {
      posicion4 = angulo_pasado;
    }
    else if (id==8)
    {
      posicion8 = angulo_pasado;
    }
    imprimirInfoMotores();
  }
  delay(10);
  motor1.setPosVel (posicion1, 15);
  delay(10);
  motor2.setPosVel (posicion2, 15);
  delay(10);
  motor4.setPosVel (posicion4, 15);
  delay(10);
  motor8.setPosVel (posicion8, 10);
  delay(10);
}