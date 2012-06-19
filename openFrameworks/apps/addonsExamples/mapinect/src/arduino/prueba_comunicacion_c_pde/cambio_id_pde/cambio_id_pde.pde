#include <ax12.h>

AX12 motor;

void setup() {
 
  motor = AX12();

  Serial.begin (115200);  // inicializa el Serial a 115,2 Kb/s
  AX12::init (1000000);   // inicializa los AX12 a 1 Mb/s

  byte detect;          
  byte num = AX12::autoDetect (&detect, 1); // detección de IDs

  Serial.print (" deteccion: ");
  Serial.println (num, DEC);

  motor.id = detect; // asigna las ID detectadas a los motores definidos previamente
  motor.SRL = RETURN_ALL;
  
  Serial.print (" ID detectado: ");
  Serial.println (detect, DEC);
  Serial.print (" delay: ");
  AX12info test = motor.readInfo (RETURN_DELAY_TIME);
  Serial.println (test.value, DEC);
  Serial.print (" error lectura: ");
  Serial.println (test.error, DEC);

}

void loop() {

    int pos = motor.getPos(); 
    int vel = motor.getSpeed(); 
    int load = motor.getLoad();
    motor.changeID(4);
    motor.setVel (30);
    motor.setPos (512);
  
    Serial.println (" ");
    Serial.print (" posicion: ");
    Serial.println (pos, DEC);
    Serial.print (" velocidad: ");
    Serial.println (vel, DEC);
    Serial.print (" carga: ");
    Serial.println (load, DEC);
        
    delay (1100);
  
}
