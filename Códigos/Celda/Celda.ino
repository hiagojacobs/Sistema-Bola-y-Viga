#include <HX711.h>

#include "HX711.h"
#define calibration_factor -2220.0

#define DOUT 8
#define CLK 5
float value;
HX711 scale;

void setup() {
 
  Serial.begin(9600);
  pinMode(13,OUTPUT);
  scale.set_scale(calibration_factor); 
  scale.tare(); 
  }

void loop() {
  Serial.print("Reading: ");
  value=scale.get_units(), 0;
  
  Serial.print(value); 
  Serial.println();
  
  delay(250);
  if(value>=2000){
  digitalWrite(13,HIGH);
  }
  else{
  digitalWrite(13,LOW);
  }
 }
