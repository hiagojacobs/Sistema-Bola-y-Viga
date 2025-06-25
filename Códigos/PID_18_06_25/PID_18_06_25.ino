#include "Adafruit_VL53L1X.h"
#include <Wire.h>

#define IRQ_PIN 8
#define XSHUT_PIN 9
#define in3Pin 10
#define in4Pin 11
#define pwmEnablePin 7
#define conA 2
#define conB 3

// Encoder
volatile int estado, estadoac;
volatile long contadore = 0;
const float cona = 0.09;
float ang = 0;

// Sensor de distancia
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// Calibración
const int CALIBRATION_OFFSET = 230;
const double ZONA_MUERTA = 0.5;
const float LIMITE_ANGULO_POS = 15.0;
const float LIMITE_ANGULO_NEG = -15.0;

// PID Externo (posición)
const double Ts2 = 0.05;
const double Kp2 = 6.42329499692026;
const double Ki2 = 6.72924801609989;
const double Kd2 = 1.13945526254747;
const double N2  = 8.45181133641681;
double a0_2, a1_2, a2_2;
double e2_k = 0, e2_k_1 = 0, e2_k_2 = 0;
double u2_k = 0;

// PID Interno (ángulo)
const double Ts1 = 0.05;
const double Kp1 = 0.00690183837349875;
const double Ki1 = 0.000137750813502502;
const double Kd1 = 0.0767578613269262;
const double N1  = 36.2891692501642;
double a0_1, a1_1, a2_1;
double e1_k = 0, e1_k_1 = 0, e1_k_2 = 0;
double u1_k = 0;

// Referencias
double Ref_Posicion = 0.0;
double Ref_Angulo = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!vl53.begin(0x29, &Wire)) {
    Serial.println("Error al iniciar sensor.");
    while (1);
  }

  if (!vl53.startRanging()) {
    Serial.println("Error al empezar ranging.");
    while (1);
  }

  vl53.setTimingBudget(50);

  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(pwmEnablePin, OUTPUT);
  pinMode(conA, INPUT_PULLUP);
  pinMode(conB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(conA), actuacoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(conB), actuacoder, CHANGE);

  // PID externo
  double ad2 = Kd2 * N2;
  double bd2 = ad2 * Ts2;
  a0_2 = Kp2 + Ki2 * Ts2 + bd2;
  a1_2 = -Kp2 - 2 * bd2;
  a2_2 = bd2;

  // PID interno
  double ad1 = Kd1 * N1;
  double bd1 = ad1 * Ts1;
  a0_1 = Kp1 + Ki1 * Ts1 + bd1;
  a1_1 = -Kp1 - 2 * bd1;
  a2_1 = bd1;
}

void loop() {
  if (vl53.dataReady()) {
    int16_t distance = vl53.distance();
    if (distance == -1) return;

    // ---------- PID EXTERNO ----------
    int y_pos = distance - CALIBRATION_OFFSET;
    e2_k_2 = e2_k_1;
    e2_k_1 = e2_k;
    e2_k = Ref_Posicion - y_pos;

    u2_k = a0_2 * e2_k + a1_2 * e2_k_1 + a2_2 * e2_k_2;
    Ref_Angulo = constrain(u2_k, LIMITE_ANGULO_NEG, LIMITE_ANGULO_POS);

    // ---------- PID INTERNO ----------
    double error_ang = Ref_Angulo - ang;

    if (abs(error_ang) < ZONA_MUERTA) {
      detenerMotor();
      e1_k = e1_k_1 = e1_k_2 = 0;
      u1_k = 0;
      Serial.println("Zona muerta - motor detenido.");
    } else {
      e1_k_2 = e1_k_1;
      e1_k_1 = e1_k;
      e1_k = error_ang;

      u1_k = a0_1 * e1_k + a1_1 * e1_k_1 + a2_1 * e1_k_2;
      u1_k = constrain(u1_k, -100, 100);
      int pwm = constrain(abs(u1_k), 0, 255);

      // Protección mínima
      if (pwm > 0 && pwm < 10) pwm = 10;

      moverMotor(u1_k > 0 ? 1 : -1, pwm);

      // ---------- DEBUG ----------
      Serial.print("Distancia(mm): "); Serial.print(y_pos);
      Serial.print(" | E2: "); Serial.print(e2_k);
      Serial.print(" | RefAng: "); Serial.print(Ref_Angulo);
      Serial.print(" | E1: "); Serial.print(error_ang);
      Serial.print(" | Out: "); Serial.print(u1_k);
      Serial.print(" | PWM: "); Serial.println(pwm);
    }

    vl53.clearInterrupt();
  }
}

// ---------- CONTROL MOTOR ----------
void moverMotor(int direccion, int pwm) {
  if (direccion > 0 && ang < LIMITE_ANGULO_POS) {
    digitalWrite(in3Pin, HIGH);
    digitalWrite(in4Pin, LOW);
  } else if (direccion < 0 && ang > LIMITE_ANGULO_NEG) {
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, HIGH);
  } else {
    detenerMotor();
    return;
  }
  analogWrite(pwmEnablePin, pwm);
}

void detenerMotor() {
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
  analogWrite(pwmEnablePin, 0);
}

// ---------- ENCODER ----------
void actuacoder() {
  if ((digitalRead(conA) == HIGH) && (digitalRead(conB) == HIGH)) estadoac = 1;
  if ((digitalRead(conA) == LOW) && (digitalRead(conB) == HIGH)) estadoac = 4;
  if ((digitalRead(conA) == LOW) && (digitalRead(conB) == LOW)) estadoac = 3;
  if ((digitalRead(conA) == HIGH) && (digitalRead(conB) == LOW)) estadoac = 2;

  switch (estadoac) {
    case 1: if (estado == 2) contadore++; if (estado == 4) contadore--; break;
    case 2: if (estado == 3) contadore++; if (estado == 1) contadore--; break;
    case 3: if (estado == 4) contadore++; if (estado == 2) contadore--; break;
    case 4: if (estado == 1) contadore++; if (estado == 3) contadore--; break;
  }

  estado = estadoac;
  ang = contadore * cona;
}
