#include "Adafruit_VL53L1X.h"
#include <ComponentObject.h>

#define IRQ_PIN 8
#define XSHUT_PIN 9
#define in3Pin 10
#define in4Pin 11
#define pwmEnablePin 7
#define conA 2
#define conB 3

const float cona = 0.09; // Conversión de pulsos a ángulos (360/4000)
volatile int estado, estadoac;
volatile long contadore = 0;
float ang = 0;

int pwm1 = 100;
int pwm2 = 0;

// Inicialización del sensor de distancia
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// Variables de control
double x0 = 0, x1 = 0, x2 = 0;
double y1 = 0, y2 = 0;
double control_output1 = 0;
double control_output2 = 0;
double Ref_Posicion = 230;

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

  // Configuración de pines para el motor y encoder
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(pwmEnablePin, OUTPUT);
  pinMode(conA, INPUT_PULLUP);
  pinMode(conB, INPUT_PULLUP);

  // Configuración de interrupciones para el encoder
  attachInterrupt(digitalPinToInterrupt(conA), actuacoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(conB), actuacoder, CHANGE);

  // Determinar el estado inicial del encoder
  if ((digitalRead(conA) == HIGH) && (digitalRead(conB) == HIGH)) estado = 1;
  if ((digitalRead(conA) == LOW) && (digitalRead(conB) == HIGH)) estado = 2;
  if ((digitalRead(conA) == LOW) && (digitalRead(conB) == LOW)) estado = 3;
  if ((digitalRead(conA) == HIGH) && (digitalRead(conB) == LOW)) estado = 4;
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

    x0 = Ref_Posicion - distance;
    control_output1 = 3.108 * x0 - 6.168*1e6 * x1 + 3.061*1e6 * x2 + 1.991 * y1 - 3.061*1e6 * y2;
    control_output2 = 14.48 * x0 - 28.94 * x1 + 14.46 * x2 + 0.8304 * y1 + 0.1696*1e6 * y2;

    x2 = x1;
    x1 = x0;
    y2 = y1;
    y1 = control_output1;

    double control_signal = control_output1;
    control_signal = constrain(control_signal, -80, 80);

    // Control del motor basado en la señal de control
    digitalWrite(pwmEnablePin, HIGH);

    if (control_signal > 0) {
      izquierda();
      analogWrite(in3Pin, control_signal);
    } else if (control_signal < 0) {
      derecha();
      analogWrite(in4Pin, -control_signal);
    } else {
      detenerMotor();
    }

    // Impresión de datos
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.print(" mm | Control signal: ");
    Serial.print(control_signal);
    Serial.print(" | Encoder Angle: ");
    Serial.println(ang);

    vl53.clearInterrupt();
  }
}

// Funciones de dirección del motor
void izquierda() {
  analogWrite(in3Pin, pwm1);
  analogWrite(in4Pin, pwm2);
}

void derecha() {
  analogWrite(in4Pin, pwm1);
  analogWrite(in3Pin, pwm2);
}

void detenerMotor() {
  analogWrite(in3Pin, 0);
  analogWrite(in4Pin, 0);
  digitalWrite(pwmEnablePin, LOW);
}

// Función de interrupción para el encoder
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
  ang = contadore * cona;  // Conversión a ángulo
}