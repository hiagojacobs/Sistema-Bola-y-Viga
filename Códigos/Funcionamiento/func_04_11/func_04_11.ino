#include <ComponentObject.h>
#include "Adafruit_VL53L1X.h"

#define IRQ_PIN 8
#define XSHUT_PIN 9

#define in3Pin 10
#define in4Pin 11
#define motorPwmPin 7

int targetPosition = 150; // Posición deseada en mm para la bola
int distance;
float ang = 0;

const byte conA = 2;
const byte conB = 3;
const float cona = 0.09; // Factor de conversión para el encoder
volatile int estado, estadoac;
volatile long contadore = 0;

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// Variables PID manual
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;  // Ajusta estos valores para tu sistema
double previousError = 0, integral = 0;

void setup() {
  // Configuración del motor
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(motorPwmPin, OUTPUT);

  // Configuración del encoder
  pinMode(conA, INPUT_PULLUP);
  pinMode(conB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(conA), actuacoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(conB), actuacoder, CHANGE);

  // Configuración del sensor de distancia
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("Inicializando el sensor de distancia VL53L1X"));
  Wire.begin();
  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error en la inicialización del sensor VL53L1X: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  
  if (!vl53.startRanging()) {
    Serial.print(F("No se pudo iniciar el rango: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }

  vl53.setTimingBudget(50);
  Serial.println(F("Sensor de distancia listo!"));

  // Configuración del Setpoint
  Setpoint = targetPosition;
}

void loop() {
  // Leer distancia desde el sensor
  if (vl53.dataReady()) {
    distance = vl53.distance();
    vl53.clearInterrupt();

    if (distance == -1) {
      Serial.println(F("Error al leer la distancia."));
    } else {
      // Asigna la posición medida como entrada para el PID manual
      Input = distance;

      // Calcular el PID manualmente
      double error = Setpoint - Input;
      integral += error;
      double derivative = error - previousError;
      Output = Kp * error + Ki * integral + Kd * derivative;
      previousError = error;

      // Limitar el valor de Output entre -255 y 255
      if (Output > 255) Output = 255;
      if (Output < -255) Output = -255;

      // Control del motor en función de la salida calculada del PID manual
      if (Output > 0) {
        moverDerecha(Output);  // Si Output es positivo, mueve a la derecha
      } else {
        moverIzquierda(-Output); // Si Output es negativo, mueve a la izquierda
      }

      // Imprimir distancia y ángulo para monitoreo
      Serial.print("Distancia: ");
      Serial.print(distance);
      Serial.print(" mm | Ángulo: ");
      Serial.println(ang);
    }
  }
}

void moverIzquierda(int pwmValue) {
  digitalWrite(in3Pin, 0);
  digitalWrite(in4Pin, 1);
  analogWrite(motorPwmPin, pwmValue);
}

void moverDerecha(int pwmValue) {
  digitalWrite(in3Pin, 1);
  digitalWrite(in4Pin, 0);
  analogWrite(motorPwmPin, pwmValue);
}

void detenerMotor() {
  analogWrite(motorPwmPin, 0);
}

void actuacoder() {
  // Actualiza el estado del encoder
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
  ang = contadore * cona;  // Convierte el conteo en ángulo
}
