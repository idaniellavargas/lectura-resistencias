//const int portPin = 35;
//// Almacenamiento del valor de puerto (Rango de 0-4095)
//int potValor = 0;
//void setup() {
//  Serial.begin(115200);
//  delay(1000);
//}
//void loop() {
//  // Lectura del valor en cada vuelta del bucle
//  potValor = analogRead(portPin);
//  Serial.println(potValor);  //Envío del valor al puerto serie
//  delay(1000);
//}
#include <ESP32Servo.h>
#include <analogWrite.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>
#include "BluetoothSerial.h"

Servo miServo1;
Servo miServo2;
Servo miServo3;
Servo miServo4;
Servo miServo5;

const float VCC=3.3; //Voltaje de la fuente
const float R2=47000.0; //2da resistencia

void setup() {
    miServo1.attach(25); //Pin del servo 1
    miServo2.attach(26); //Pin del servo 2
    miServo3.attach(27); //Pin del servo 3
    miServo4.attach(14); //Pin del servo 4
    miServo5.attach(12); //Pin del servo 5

  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(35,INPUT); // Pin del sensor flex 1
pinMode(35,INPUT); // Pin del sensor flex 2
pinMode(35,INPUT); // Pin del sensor flex 3
pinMode(35,INPUT); // Pin del sensor flex 4
pinMode(35,INPUT); // Pin del sensor flex 5

}

void loop() {
  // put your main code here, to run repeatedly:
int ADC_flex1=analogRead(35); //Lectura del sensor flex 1
int ADC_flex2=analogRead(35); //Lectura del sensor flex 2
int ADC_flex3=analogRead(35); //Lectura del sensor flex 3
int ADC_flex4=analogRead(35); //Lectura del sensor flex 4
int ADC_flex5=analogRead(35); //Lectura del sensor flex 5

float Vflex1=ADC_flex1*VCC/4095.0;
float Rflex1=R2*(VCC/Vflex1-1.0);

float Vflex2=ADC_flex2*VCC/4095.0;
float Rflex2=R2*(VCC/Vflex2-1.0);

float Vflex3=ADC_flex3*VCC/4095.0;
float Rflex3=R2*(VCC/Vflex3-1.0);

float Vflex4=ADC_flex4*VCC/4095.0;
float Rflex4=R2*(VCC/Vflex4-1.0);

float Vflex5=ADC_flex5*VCC/4095.0;
float Rflex5=R2*(VCC/Vflex5-1.0);

//Serial.println("Resistencia: " + String(Rflex) + "ohms");

float angle1 = map( Rflex1,13000.0,35000.0,0,90.0);
float angle2 = map( Rflex2,13000.0,35000.0,0,90.0);
float angle3 = map( Rflex3,13000.0,35000.0,0,90.0); //FIJO
float angle4 = map( Rflex4,13000.0,35000.0,0,90.0);
float angle5 = map( Rflex5,13000.0,35000.0,0,90.0);

//Serial.println(angle);
miServo1.write(120-(angle1*(11.0/9.0)));
miServo2.write(140-(angle2*(14.0/9.0)));
miServo3.write(140-(angle3*(14.0/9.0)));
miServo4.write(65+(angle4*(11.5/9.0)));
miServo5.write(65+(angle5*(11.5/9.0)));

//Serial.println(140-(angle*(14.0/9.0)));
delay(50);
}

//angulo abierto - (angulo*(Angulo abierto/90)) 
//Valor flotante .0
