#include "Adafruit_VL53L1X.h"
#include <ComponentObject.h>

#define IRQ_PIN 8
#define XSHUT_PIN 9
#define in3Pin 10         // Dirección motor
#define in4Pin 11         // Dirección motor
#define pwmEnablePin 7    // Pin PWM
#define conA 2            // Encoder canal A
#define conB 3            // Encoder canal B

const float cona = 0.09;  // Conversión de pulsos a ángulos (360/4000)
volatile int estado, estadoac;
volatile long contadore = 0;
float ang = 0;

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);
double control_output = 0;
double Ref_Posicion = 0;  // 0 es la referencia tras calibración

const double Kp = 3;
const double Ki = 0.5;     // Ganancia integral
double error_integral = 0; // Acumulador de la integral del error

const int CALIBRATION_OFFSET = 230;  // 230 mm será el nuevo 0
const int ZONA_MUERTA = 3;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("Adafruit VL53L1X sensor demo"));
  Wire.begin();

  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }

  Serial.println(F("VL53L1X sensor OK!"));
  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }

  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(pwmEnablePin, OUTPUT);
  pinMode(conA, INPUT_PULLUP);
  pinMode(conB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(conA), actuacoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(conB), actuacoder, CHANGE);
}

void loop() {
  int16_t distance;

  if (vl53.dataReady()) {
    distance = vl53.distance();

    if (distance == -1) {
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return;
    }

    int adjusted_distance = distance - CALIBRATION_OFFSET;
    double error = Ref_Posicion - adjusted_distance;

    Serial.print(F("Distance (raw): "));
    Serial.print(distance);
    Serial.print(" mm | Adjusted Distance: ");
    Serial.print(adjusted_distance);
    Serial.print(" mm | Error: ");
    Serial.println(error);

    if (abs(error) < ZONA_MUERTA) {
      detenerMotor();
      Serial.println(F("Dentro de la zona muerta, motor detenido."));
      error_integral = 0;  // Resetear el acumulador si se detiene
      return;
    }

    // Acumulación del error para el término integral
    error_integral += error;

    // Control PI
    control_output = Kp * error + Ki * error_integral;

    control_output = constrain(control_output, -80, 80);

    int pwm_value = abs(control_output);
    if (pwm_value > 0 && pwm_value < 40) pwm_value = 40;

    if (control_output > 0) {
      moverMotor(1, pwm_value);
    } else {
      moverMotor(-1, pwm_value);
    }

    Serial.print(F("Control output: "));
    Serial.print(control_output);
    Serial.print(" | PWM: ");
    Serial.print(pwm_value);
    Serial.print(" | Encoder Angle: ");
    Serial.println(ang);

    vl53.clearInterrupt();
  }
}

void moverMotor(int direccion, int pwm) {
  if (direccion > 0) {
    digitalWrite(in3Pin, HIGH);
    digitalWrite(in4Pin, LOW);
  } else {
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, HIGH);
  }
  analogWrite(pwmEnablePin, pwm);
}

void detenerMotor() {
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
  analogWrite(pwmEnablePin, 0);
}

void actuacoder() {
  if ((digitalRead(conA) == HIGH) && (digitalRead(conB) == HIGH)) estadoac = 1;
  if ((digitalRead(conA) == LOW) && (digitalRead(conB) == HIGH)) estadoac = 4;
  if ((digitalRead(conA) == LOW) && (digitalRead(conB) == LOW)) estadoac = 3;
  if ((digitalRead(conA) == HIGH) && (digitalRead(conB) == LOW)) estadoac = 2;

  switch (estadoac) {
    case 1:
      if (estado == 2) contadore++;
      if (estado == 4) contadore--;
      break;
    case 2:
      if (estado == 3) contadore++;
      if (estado == 1) contadore--;
      break;
    case 3:
      if (estado == 4) contadore++;
      if (estado == 2) contadore--;
      break;
    case 4:
      if (estado == 1) contadore++;
      if (estado == 3) contadore--;
      break;
  }

  estado = estadoac;
  ang = contadore * cona;
}
