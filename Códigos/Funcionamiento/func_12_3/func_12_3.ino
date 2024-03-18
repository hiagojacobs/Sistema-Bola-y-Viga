#include "Adafruit_VL53L1X.h"

#define IRQ_PIN 8
#define XSHUT_PIN 9

#define in3Pin 11
#define in4Pin 10
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

int16_t pwm = 72;
int16_t P = 0.49;
void setup() {
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
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  /*
  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  vl.VL53L1X_SetInterruptPolarity(0);
  */
}

void loop() {
  int16_t distance;
  int16_t control; 
  
  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    control = distance - 230;
    if (distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return;
    }
    if (control >= 20){
      //pwm = control * 1.5 * 0.29;
      digitalWrite(in3Pin, 1);
      digitalWrite(in4Pin, 0);
      analogWrite(6,65);


      }
      else if(control <= 20){
      //pwm = control * -1.13 * 0.29;
      digitalWrite(in3Pin, 0);
      digitalWrite(in4Pin, 1);
      analogWrite(6,72);

      }
      else{
        digitalWrite(in3Pin, 0);
        digitalWrite(in4Pin, 0);
      }
    
    Serial.print(F("Distance: "));
    Serial.print(control);
    Serial.println(" mm");

    // data is read out, time for another reading!
    vl53.clearInterrupt();
  }
}
