#include "Adafruit_VL53L1X.h"

#define IRQ_PIN 2
#define XSHUT_PIN 3
#define in3Pin 2 // Conecta In3 del puente H al pin 2 de Arduino
#define in4Pin 1  // Conecta In4 del puente H al pin 1 de Arduino

int pwm=90;
int pwm2=80;

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

void setup() {
  //Motor
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);

  //Sensor distancia
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("Adafruit VL53L1X sensor demo"));

  Wire.begin();
  if (! vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

}

void loop() {
  //digitalWrite(in3Pin, 1);
  //digitalWrite(in4Pin, 0);
  //analogWrite(6,pwm2);
  //delay(150);
  
  // Detiene el motor durante 1 segundo
  //digitalWrite(in3Pin, 0);
  //digitalWrite(in4Pin, 1);
  //analogWrite(6,pwm);
  //delay(150);

  int16_t distance;

  if (vl53.dataReady()) {

    distance = vl53.distance();
    if (distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return;
    }
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(" mm");

    // data is read out, time for another reading!
    vl53.clearInterrupt();
  
  }

  if(distance > 200){
    distance = vl53.distance();
    digitalWrite(in3Pin, 1);
    digitalWrite(in4Pin, 0);
    analogWrite(6,pwm2);
    delay(150);
  
    digitalWrite(in3Pin, 0);
    digitalWrite(in4Pin, 1);
    analogWrite(6,pwm);
    delay(150);
  }
}