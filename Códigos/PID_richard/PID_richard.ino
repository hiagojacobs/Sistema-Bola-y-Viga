#include "Adafruit_VL53L1X.h"
#include <ComponentObject.h>

#define IRQ_PIN 8
#define XSHUT_PIN 9
#define in3Pin 10
#define in4Pin 11
#define pwmEnablePin 7
#define conA 2
#define conB 3

const float cona = 0.09;
volatile int estado, estadoac;
volatile long contadore = 0;
float ang = 0;

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);
double control_output = 0;
double Ref_Posicion = 0;

// 🔧 Ganancias PID ajustadas para mayor respuesta
const double Kp = 4.0;
const double Ki = 0.1;
const double Kd = 1.5;

double prev_error = 0;
double prev_prev_error = 0;
double control_output_prev = 0;

const int CALIBRATION_OFFSET = 230;
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
      return;
    }

    // PID con ecuación en diferencias
    double delta_error = error - prev_error;
    double delta_error2 = error - 2 * prev_error + prev_prev_error;

    control_output = control_output_prev
                   + Kp * delta_error
                   + Ki * error
                   + Kd * delta_error2;

    prev_prev_error = prev_error;
    prev_error = error;
    control_output_prev = control_output;

    // 🎯 PWM proporcional con nuevo rango ±200 mm → ±80 PWM
    double max_output = 200.0;
    control_output = constrain(control_output, -max_output, max_output);
    double pwm_mapped = (control_output / max_output) * 80.0;

    if (pwm_mapped > 0) {
      moverMotor(1, (int)pwm_mapped);
    } else if (pwm_mapped < 0) {
      moverMotor(-1, (int)(-pwm_mapped));
    } else {
      detenerMotor();
    }

    Serial.print(F("Control output: "));
    Serial.print(control_output);
    Serial.print(" | PWM mapped: ");
    Serial.print(pwm_mapped);
    Serial.print(" | Adjusted Dist: ");
    Serial.print(adjusted_distance);
    Serial.print(" | Error: ");
    Serial.print(error);
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
