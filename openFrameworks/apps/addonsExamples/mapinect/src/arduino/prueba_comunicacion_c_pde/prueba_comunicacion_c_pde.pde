#include <ax12.h>

AX12 motor1;
AX12 motor2;
AX12 motor4;
AX12 motor8;

int posicion1 = 512;
int posicion2 = 512;
int posicion4 = 512;
int posicion8 = 582; //el cero está corrido :@

int posicion1verdadera = 512;
int posicion2verdadera = 512;
int posicion4verdadera = 512;
int posicion8verdadera = 582; //el cero está corrido :@

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
  byte mask = B10000000;
  int direccion = 1;
  if (mask & angulo)
  {
    direccion = -1;
  }
  mask = B01111111;
  angulo = mask & angulo;
  return 512 + direccion * angulo * (1024.0 / 300.0);    
}

void imprimirInfoMotores()
{
  Serial.println("Motor 1:");
  int pos = motor1.getPos(); 
  int load = motor1.getLoad();
  Serial.print("Posicion: ");
  Serial.println(pos, BIN);
  Serial.println(pos, DEC);
  Serial.print("Carga: ");
  Serial.println(load, DEC);
  
  Serial.println("Motor 2:");
  pos = motor2.getPos(); 
  load = motor2.getLoad();
  Serial.print("Posicion: ");
  Serial.println(pos, BIN);
  Serial.println(pos, DEC);
  Serial.print("Carga: ");
  Serial.println(load, DEC);
  
  Serial.println("Motor 4:");
  pos = motor4.getPos(); 
  load = motor4.getLoad();
  Serial.print("Posicion: ");
  Serial.println(pos, BIN);
  Serial.println(pos, DEC);
  Serial.print("Carga: ");
  Serial.println(load, DEC);
  
  Serial.println("Motor 8:");
  pos = motor8.getPos(); 
  load = motor8.getLoad();
  Serial.print("Posicion: ");
  Serial.println(pos, BIN);
  Serial.println(pos, DEC);
  Serial.print("Carga: ");
  Serial.println(load, DEC);
}

void llegoAPosicionFinal(){
  int p1va = motor1.getPos();
  int p2va = motor2.getPos();
  int p4va = motor4.getPos();
  int p8va = motor8.getPos();
  if (p1va != posicion1verdadera || p2va != posicion2verdadera || p4va != posicion4verdadera || p8va != posicion8verdadera){
    posicion1verdadera = p1va;
    posicion2verdadera = p2va;
    posicion4verdadera = p4va;
    posicion8verdadera = p8va;
    boolean llegoMotor1 = ((posicion1verdadera - posicion1) * (posicion1verdadera - posicion1) <= 16);
    boolean llegoMotor2 = ((posicion2verdadera - posicion2) * (posicion2verdadera - posicion2) <= 16);
    boolean llegoMotor4 = ((posicion4verdadera - posicion4) * (posicion4verdadera - posicion4) <= 16);
    boolean llegoMotor8 = ((posicion8verdadera - posicion8) * (posicion8verdadera - posicion8) <= 16);
    Serial.println(llegoMotor1 && llegoMotor2 && llegoMotor4 && llegoMotor8);
    imprimirInfoMotores();
  }
}

void loop()
{
  if (Serial.available() > 1)
  {
    byte id = Serial.read();
    byte angulo = Serial.read();
    if (id == 0){
      llegoAPosicionFinal();
    }
    else{
      int angulo_pasado = pasarAngulo(angulo);
      if (id==1){
        posicion1 = angulo_pasado;
      }
      else if (id==2){
        posicion2 = angulo_pasado;
      }
      else if (id==4)
      {
        posicion4 = angulo_pasado;
      }
      else if (id==8)
      {
        posicion8 = angulo_pasado + 70;//el cero está corrido :@
      }
    }
  }
  delay(5);
  motor1.setPosVel (posicion1, 15);
  delay(5);
  motor2.setPosVel (posicion2, 30);
  delay(5);
  motor4.setPosVel (posicion4, 30);
  delay(5);
  motor8.setPosVel (posicion8, 20);
}
