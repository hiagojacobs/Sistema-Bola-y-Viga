#include "Adafruit_VL53L1X.h"
#include <Wire.h>

// Pines hardware
#define IRQ_PIN 8
#define XSHUT_PIN 9
#define in3Pin 10
#define in4Pin 11
#define pwmEnablePin 7
#define conA 2
#define conB 3

// Encoder y variables
volatile long encoder_ticks = 0;
float vel_actual = 0;
float ang = 0;
const float pulsos_por_grado = 0.09;

long encoder_prev = 0;
int pwm_prev = 0;

// Sensor
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// Controladores
const float Kp_pos = 0.0204933175108283;
const float Kp_vel = 3.856202245096381;

const int CALIBRATION_OFFSET = 230;
const int ZONA_MUERTA_POS = 3;
const int ZONA_MUERTA_VEL = 1;

float Ref_Pos = 0; // en mm
float Ref_Vel = 0; // en grados/seg

unsigned long t_prev = 0;

// ------------ SETUP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!vl53.begin(0x29, &Wire)) {
    Serial.println("Error VL53L1X");
    while (1);
  }
  vl53.setTimingBudget(50);
  vl53.startRanging();

  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(pwmEnablePin, OUTPUT);
  pinMode(conA, INPUT_PULLUP);
  pinMode(conB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(conA), encoder_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(conB), encoder_ISR, CHANGE);

  t_prev = millis();
}

// ------------ LOOP -----------------
void loop() {
  unsigned long t_now = millis();
  if (t_now - t_prev >= 50) {
    t_prev = t_now;

    // --- Leer posición ---
    int16_t dist = vl53.distance();
    if (dist == -1) return;
    vl53.clearInterrupt();

    int pos_mm = dist - CALIBRATION_OFFSET;
    float error_pos = Ref_Pos - pos_mm;

    // --- Control externo ---
    if (abs(error_pos) < ZONA_MUERTA_POS) Ref_Vel = 0;
    else Ref_Vel = Kp_pos * error_pos;
    Ref_Vel = constrain(Ref_Vel, -180, 180);

    // --- Calcular velocidad actual ---
    long delta_ticks = encoder_ticks - encoder_prev;
    vel_actual = (delta_ticks * pulsos_por_grado) / 0.05;
    encoder_prev = encoder_ticks;

    // --- Control interno ---
    float error_vel = Ref_Vel - vel_actual;
    int pwm_cmd = 0;

    if (abs(error_vel) < ZONA_MUERTA_VEL) {
      detenerMotor();
    } else {
      pwm_cmd = Kp_vel * error_vel;
      pwm_cmd = constrain(pwm_cmd, -255, 255);

      // APLICAR SUAVIZADO
      int pwm_suavizado = suavizarPWM(abs(pwm_cmd), pwm_prev, 10);
      pwm_prev = pwm_suavizado;

      moverMotor(pwm_cmd > 0 ? 1 : -1, pwm_suavizado);
    }

    // --- Debug ---
    Serial.print("Pos(mm): "); Serial.print(pos_mm);
    Serial.print(" | Vref: "); Serial.print(Ref_Vel);
    Serial.print(" | Vact: "); Serial.print(vel_actual);
    Serial.print(" | PWM: "); Serial.println(pwm_cmd);
  }
}

// ------------ MOTOR -------------
void moverMotor(int dir, int pwm) {
  digitalWrite(in3Pin, dir > 0);
  digitalWrite(in4Pin, dir < 0);
  analogWrite(pwmEnablePin, pwm);
}

void detenerMotor() {
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
  analogWrite(pwmEnablePin, 0);
}

// ------------ ENCODER ISR -------------
void encoder_ISR() {
  static int last_A = 0, last_B = 0;
  int A = digitalRead(conA);
  int B = digitalRead(conB);
  int delta = (last_A == A && last_B != B) || (last_A != A && last_B == B) ? 1 : -1;
  encoder_ticks += delta;
  last_A = A;
  last_B = B;
}

// ------------ PWM SUAVIZADO -------------
int suavizarPWM(int actual, int anterior, int max_delta) {
  int delta = actual - anterior;
  if (abs(delta) > max_delta)
    return anterior + (delta > 0 ? max_delta : -max_delta);
  else
    return actual;
}
